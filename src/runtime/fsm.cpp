#include "icar.hpp"

void Icar::runFsm(Mat &img)
    {
        params->geometryPolicy = GeometryPolicy::PERCEPTION_ALLOWED;
        // 圈数变更时复位所有FSM状态
        if (lastLap != params->currentLap)
        {
            lastLap = params->currentLap;
            fsmFactory.stop->resetLap();
            fsmFactory.park->resetLap();
            fsmFactory.fork->resetLap();
            fsmFactory.yfork->resetLap();
            fsmFactory.slow->resetLap();
            fsmFactory.busy->resetLap();
            fsmFactory.station->resetLap();
            fsmFactory.obstacle->resetLap();
            params->plannerSafety.clear();
            params->setStopReason(control_algorithms::StopReason::PLANNER, false);
            params->alertCountdown = 0;                // 复位蜂鸣器报警
            params->alertDecelCount = 0;               // 复位报警减速
            params->busyAlertCountdown = 0;            // 复位施工区蜂鸣
        }

        if (params->mode == FsmMode::STATION ||
            params->mode == FsmMode::FORK ||
            params->mode == FsmMode::BUSY ||
            params->mode == FsmMode::SLOW ||
            params->mode == FsmMode::YFORK) // 状态复位
            params->mode = FsmMode::NORMAL;

        // 处理手动接管（优先执行，跳过所有FSM检测）
        // Monitoring connection does not trigger takeover.

        params->manualTakeover = fsmFactory.busy->isInManualTakeover();
        if (params->manualTakeover)
        {
            // 手动接管期间把停车类停止原因交还驾驶员：接管结束后由各FSM
            // 根据当前阶段重新断言，避免 PARK_GATE/PARK_TARGET_LOST/PARK
            // 残留在手动控制链路中强制停车。
            params->setStopReason(control_algorithms::StopReason::PARK, false);
            params->setStopReason(
                control_algorithms::StopReason::PARK_GATE, false);
            params->setStopReason(
                control_algorithms::StopReason::PARK_TARGET_LOST, false);
            // 检查是否返回自动模式
            if (fsmFactory.manual->checkForReturnKey())
            {
                fsmFactory.busy->endManualTakeover();
                params->setStopReason(control_algorithms::StopReason::MANUAL, false);
                params->mode = FsmMode::NORMAL;   // 恢复自动模式
                params->takeoverJustEnded = true; // 通知各FSM手动接管刚结束
                params->autoRecoveryFrames = 2;
                params->setStopReason(control_algorithms::StopReason::STARTUP, true);
                params->ctrl.speed = 0.0f;
                params->ctrl.servo = PWMSERVOMID;
            }
            else if (fsmFactory.manual->isConnected() &&
                     fsmFactory.manual->isManualControl())
            {
                fsmFactory.manual->applyManualControl(&params->ctrl.speed, &params->ctrl.servo);
                params->setStopReason(control_algorithms::StopReason::MANUAL, false);
            }
            else
            {
                params->setStopReason(control_algorithms::StopReason::MANUAL, true);
                params->ctrl.speed = 0;
                params->ctrl.servo = PWMSERVOMID;
            }

            if (params->mode != params->modeLast)
            {
                client->buzzerSound(client->BUZZER_DING); // 提示音效
                params->modeLast = params->mode;
            }
            return; // 手动接管期间跳过所有FSM检测
        }

        fsmFactory.stop->run(img); // 停车区识别与规划
        params->mode = fsmFactory.stop->getMode();
        if (params->hasStopReason(control_algorithms::StopReason::GATE))
            return;
        fsmFactory.cross->run(img); // 斑马线停车识别与规划
        const FsmMode crossMode = fsmFactory.cross->getMode();
        if (crossMode != FsmMode::NORMAL)
            params->mode = crossMode;
        if (params->hasStopReason(control_algorithms::StopReason::CROSS))
            return;

        if (params->hasStopReason(control_algorithms::StopReason::PARK))
        {
            fsmFactory.park->run(img);
            params->mode = fsmFactory.park->getMode();
            return;
        }
        if (params->hasStopReason(control_algorithms::StopReason::BUSY))
        {
            fsmFactory.busy->run(img);
            params->mode = fsmFactory.busy->getMode();
            return;
        }
        if (params->hasStopReason(control_algorithms::StopReason::STATION))
        {
            fsmFactory.station->run(img);
            params->mode = fsmFactory.station->getMode();
            return;
        }

        // ===== 路线隔离：仅在当前圈使能时调用对应FSM =====
        if (params->featureEnabled(Feature::PARK))
        {
            if (params->mode == FsmMode::NORMAL || params->mode == FsmMode::PARK || params->mode == FsmMode::CROSS)
            {
                fsmFactory.park->run(img);
                FsmMode mode = fsmFactory.park->getMode();
                if (mode != FsmMode::NORMAL)
                    params->mode = mode;
                if (params->hasStopReason(control_algorithms::StopReason::PARK))
                    return;
            }
        }
        if (params->featureEnabled(Feature::FORK))
        {
            if (params->mode == FsmMode::NORMAL)
            {
                fsmFactory.fork->run(img);
                params->mode = fsmFactory.fork->getMode();
            }
        }
        if (params->featureEnabled(Feature::YFORK))
        {
            if (params->mode == FsmMode::NORMAL)
            {
                fsmFactory.yfork->run(img);
                params->mode = fsmFactory.yfork->getMode();
            }
        }
        if (params->featureEnabled(Feature::SLOW))
        {
            fsmFactory.slow->run(img);
            if (params->mode == FsmMode::NORMAL)
                params->mode = fsmFactory.slow->getMode();
        }
        if (params->featureEnabled(Feature::BUSY))
        {
            fsmFactory.busy->run(img);
            if (params->mode == FsmMode::NORMAL)
                params->mode = fsmFactory.busy->getMode();
            if (params->hasStopReason(control_algorithms::StopReason::BUSY) ||
                fsmFactory.busy->isInManualTakeover())
                return;

        }
        if (params->featureEnabled(Feature::STATION))
        {
            fsmFactory.station->run(img);
            FsmMode stationMode = fsmFactory.station->getMode();
            // 不覆盖YFORK模式，让yfork状态能正常显示
            if (stationMode != FsmMode::NORMAL && params->mode != FsmMode::YFORK)
                params->mode = stationMode;
            if (params->hasStopReason(control_algorithms::StopReason::STATION))
                return;
        }

        // 全局障碍物检测（锥桶/行人），施工区/停车场/Y型岔路口期间不检测
        params->ctrl.obstacleSlow = false;
        const bool obstacleAllowed = pathSourceAllowed(
            PathSource::OBSTACLE, params->mode);
        if (!obstacleAllowed)
            params->clearPathOverride(PathSource::OBSTACLE);
        if (params->featureEnabled(Feature::OBSTACLE) && obstacleAllowed)
            fsmFactory.obstacle->run(img);


    }
