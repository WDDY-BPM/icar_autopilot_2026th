/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习).
 *            The code ADAPTS the corresponding hardware circuit board(智能汽车-ICAR),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file boot.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 智能车自启Boot程序
 * @version 0.1
 * @date 2024-09-03
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include "com/server.hpp"
#include <string>
#include <fstream>
#include <sys/types.h>
#include <sys/wait.h>
#include <limits.h>
#include <chrono>
#include <csignal>

using namespace std;
int launchCmd(const std::string &workdir, const std::string &cmd, bool wait);

static std::string executableDirectory()
{
    char path[PATH_MAX] = {};
    const ssize_t length = readlink("/proc/self/exe", path, sizeof(path) - 1);
    if (length <= 0)
        return ".";
    path[length] = '\0';
    const std::string fullPath(path);
    const size_t separator = fullPath.find_last_of('/');
    return separator == std::string::npos ? "." : fullPath.substr(0, separator);
}

static bool processRunning(pid_t &pid)
{
    if (pid <= 0)
        return false;
    int status = 0;
    const pid_t result = waitpid(pid, &status, WNOHANG);
    if (result == 0)
        return true;
    pid = -1;
    return false;
}

static bool icarProcessExists()
{
    return std::system("pgrep -x icar >/dev/null 2>&1") == 0;
}

int main(int argc, char const *argv[])
{
    Server server;
    if (!server.start())
        return -1;

    constexpr int64_t WATCHDOG_TIMEOUT_MS = 2000;
    constexpr int64_t STARTUP_GRACE_MS = 30000;
    pid_t appPid = -1;
    bool clientWasSeen = false;
    auto appStartedAt = std::chrono::steady_clock::time_point{};

    printf("Boot is running!\n");
    while (true)
    {
        processRunning(appPid); // reap a child that exited
        if (server.isClientConnected())
            clientWasSeen = true;
        if (server.watchdogExpired(WATCHDOG_TIMEOUT_MS))
        {
            std::cerr << "[Boot] Valid-frame watchdog timed out; stopping vehicle.\n";
            server.handleWatchdogTimeout();
        }
        if (!server.isClientConnected())
            server.uart.sendHeart();

        usleep(200 * 1000);

        if (server.uart.killAll.exchange(false))
        {
            server.uart.carControl(0, 1500);
            std::system("killall -9 icar collection img2video calibration camera detection main");
            appPid = -1;
            clientWasSeen = false;
            printf("Kill all app!\n");
        }
        else if (server.uart.keypress.exchange(false))
        {
            if (server.isClientConnected())
            {
                server.transmit("Keypress");
                server.handleWatchdogTimeout();
                std::system("pkill -x icar");
                appPid = -1;
                clientWasSeen = false;
                printf("App was killed!\n");
            }
            else if (processRunning(appPid))
            {
                const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - appStartedAt).count();
                if (elapsed < STARTUP_GRACE_MS)
                {
                    printf("App is still initializing; duplicate launch blocked.\n");
                }
                else
                {
                    kill(appPid, SIGKILL);
                    waitpid(appPid, nullptr, 0);
                    appPid = -1;
                    server.uart.carControl(0, 1500);
                    printf("Unconnected app was stopped; press again to restart.\n");
                }
            }
            else if (clientWasSeen || icarProcessExists())
            {
                std::system("pkill -x icar");
                server.uart.carControl(0, 1500);
                clientWasSeen = false;
                printf("Disconnected app was stopped; press again to restart.\n");
            }
            else
            {
                appPid = launchCmd(executableDirectory(), "./icar", false);
                if (appPid > 0)
                {
                    appStartedAt = std::chrono::steady_clock::now();
                    printf("App icar is starting (30 s connection grace).\n");
                }
            }
            usleep(2000 * 1000);
        }
        else if (server.uart.exitBoot.exchange(false))
        {
            server.uart.carControl(0, 1500);
            std::system("killall -9 icar collection img2video calibration camera detection main");
            break;
        }
    }

    printf("Boot was closed!\n");
    server.closeServer();
    return 0;
}

/**
 * @brief 启动命令
 *
 * @param workdir 工作目录
 * @param cmd 命令
 * @param wait 是否等待命令执行完毕
 * @return int 命令执行状态
 */
int launchCmd(const std::string &workdir, const std::string &cmd, bool wait)
{
    pid_t pid = fork();

    if (pid == 0)
    {
        // 子进程
        if (!workdir.empty())
        {
            if (chdir(workdir.c_str()) != 0)
            {
                perror("chdir failed");
                _exit(1);
            }
        }
        execl("/bin/sh", "sh", "-c", cmd.c_str(), (char *)NULL);
        perror("execl failed");
        _exit(1);
    }
    else if (pid > 0)
    {
        // 父进程
        if (wait)
        {
            int status;
            if (waitpid(pid, &status, 0) == -1)
            {
                perror("waitpid failed");
                return -1;
            }
            if (WIFEXITED(status))
            {
                return WEXITSTATUS(status); // 返回退出码
            }
            else if (WIFSIGNALED(status))
            {
                std::cerr << "Process killed by signal " << WTERMSIG(status) << std::endl;
                return -1;
            }
            return -1;
        }
        else
        {
            return pid; // 后台运行，返回子进程pid
        }
    }
    else
    {
        perror("fork failed");
        return -1;
    }
}
