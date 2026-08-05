#include "icar.hpp"

bool Icar::scanFreshAiAlerts()
    {
        bool decelFound = false;
        if (params->config.alertTarget != "none")
        {
            int alertLabel = -1;
            if (params->config.alertTarget == "cone")
                alertLabel = LABEL_CONE;
            else if (params->config.alertTarget == "person")
                alertLabel = LABEL_PERSON;
            else if (params->config.alertTarget == "busy")
                alertLabel = LABEL_BUSY;
            else if (params->config.alertTarget == "limit")
                alertLabel = LABEL_LIMIT;
            else if (params->config.alertTarget == "unlimit")
                alertLabel = LABEL_UNLIMIT;
            else if (params->config.alertTarget == "park")
                alertLabel = LABEL_PARK;

            // 施工区由FSM自身负责鸣笛，不重复触发
            if (alertLabel == LABEL_BUSY && params->config.currentLapConfig &&
                params->config.currentLapConfig->busy)
                alertLabel = -1;

            if (alertLabel >= 0)
            {
                bool targetFound = false;
                for (const auto &result : params->results)
                {
                    const bool positionOk =
                        (alertLabel == LABEL_LIMIT || alertLabel == LABEL_PARK)
                            ? (result.x > COLSIMAGE * 0.8 ||
                               result.x + result.width < COLSIMAGE * 0.2)
                            : (result.y + result.height) > ROWSIMAGE * 0.2;
                    if (result.type == alertLabel && positionOk)
                    {
                        targetFound = true;
                        break;
                    }
                }
                control_algorithms::refreshAlertEvidence(
                    true, targetFound, params->alertCountdown);
            }
        }

        for (const auto &result : params->results)
        {
            bool positionOk = false;
            if (result.type == LABEL_LIMIT || result.type == LABEL_PARK)
                positionOk = result.x > COLSIMAGE * 0.8 ||
                             result.x + result.width < COLSIMAGE * 0.2;
            else if (result.type == LABEL_CONE || result.type == LABEL_PERSON ||
                     result.type == LABEL_UNLIMIT)
                positionOk = (result.y + result.height) > ROWSIMAGE * 0.2;
            else if (result.type == LABEL_BUSY)
            {
                if (params->config.currentLapConfig &&
                    params->config.currentLapConfig->busy)
                    continue;
                positionOk = (result.y + result.height) > ROWSIMAGE * 0.2;
            }
            else
                continue;

            if (positionOk)
            {
                decelFound = true;
                break;
            }
        }
        return decelFound;
    }

void Icar::advanceAlertTimers(bool decelEvidenceRefreshed)
    {
        const auto events = control_algorithms::advanceAlertTimers(
            params->busyAlertCountdown, params->alertCountdown,
            params->alertDecelCount, decelEvidenceRefreshed);
        if (events.busyBeep)
            client->buzzerSound(client->BUZZER_WARNNING);
        if (events.regularBeep)
            client->buzzerSound(client->BUZZER_WARNNING);
    }

void Icar::updateAlerts()
    {
        if (lastLap != params->currentLap)
        {
            params->alertCountdown = 0;
            params->alertDecelCount = 0;
            params->busyAlertCountdown = 0;
            return;
        }

        const bool decelEvidenceRefreshed = params->aiResultFresh
            ? scanFreshAiAlerts() : false;
        advanceAlertTimers(decelEvidenceRefreshed);

        if (params->mode != params->modeLast && params->alertCountdown <= 0)
        {
            client->buzzerSound(client->BUZZER_DING); // 提示音效
            params->modeLast = params->mode;
        }
    }
