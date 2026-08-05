#pragma once

#include <atomic>
#include <cstdint>

class ControlWatchdogState
{
public:
    void reset()
    {
        armed_.store(false, std::memory_order_release);
        lastControlFrameMs_.store(0, std::memory_order_relaxed);
    }

    void onConnected()
    {
        reset();
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

    bool expired(std::int64_t nowMs, std::int64_t timeoutMs) const
    {
        if (!armed())
            return false;
        return nowMs - lastControlFrameMs_.load(std::memory_order_relaxed) > timeoutMs;
    }

private:
    std::atomic<bool> armed_{false};
    std::atomic<std::int64_t> lastControlFrameMs_{0};
};
