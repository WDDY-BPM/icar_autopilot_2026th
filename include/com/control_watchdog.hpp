#pragma once

#include <atomic>
#include <cstdint>

class ControlWatchdogState
{
public:
    void reset()
    {
        connected_.store(false, std::memory_order_release);
        armed_.store(false, std::memory_order_release);
        connectedAtMs_.store(0, std::memory_order_relaxed);
        lastControlFrameMs_.store(0, std::memory_order_relaxed);
    }

    void onConnected(std::int64_t nowMs = 0)
    {
        reset();
        connectedAtMs_.store(nowMs, std::memory_order_relaxed);
        connected_.store(true, std::memory_order_release);
    }

    void onValidControlFrame(std::int64_t nowMs)
    {
        lastControlFrameMs_.store(nowMs, std::memory_order_relaxed);
        armed_.store(true, std::memory_order_release);
    }

    bool onValidFrame(std::uint8_t address, std::uint8_t controlAddress,
                      std::int64_t nowMs)
    {
        if (address != controlAddress)
            return false;
        onValidControlFrame(nowMs);
        return true;
    }

    bool armed() const
    {
        return armed_.load(std::memory_order_acquire);
    }

    bool startupExpired(std::int64_t nowMs, std::int64_t graceMs) const
    {
        if (!connected_.load(std::memory_order_acquire) || armed())
            return false;
        return nowMs - connectedAtMs_.load(std::memory_order_relaxed) > graceMs;
    }

    bool expired(std::int64_t nowMs, std::int64_t timeoutMs) const
    {
        if (!armed())
            return false;
        return nowMs - lastControlFrameMs_.load(std::memory_order_relaxed) > timeoutMs;
    }

private:
    std::atomic<bool> connected_{false};
    std::atomic<bool> armed_{false};
    std::atomic<std::int64_t> connectedAtMs_{0};
    std::atomic<std::int64_t> lastControlFrameMs_{0};
};
