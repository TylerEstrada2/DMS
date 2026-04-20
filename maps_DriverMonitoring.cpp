#include <algorithm>
#include "maps_DriverMonitoring.h"

// inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSDriverMonitoring)
    MAPS_INPUT("DMSIP_DrvrAttnStatAuth", MAPS::FilterInteger64, MAPS::SamplingReader)
    MAPS_INPUT("StrgWhlTchSnsHndsOnStat", MAPS::FilterInteger64, MAPS::SamplingReader)
    MAPS_INPUT("HMI2DMS_ActivateSignal", MAPS::FilterInteger64, MAPS::SamplingReader)
    MAPS_INPUT("ActETRS", MAPS::FilterInteger64, MAPS::SamplingReader)
MAPS_END_INPUTS_DEFINITION

// outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSDriverMonitoring)
    MAPS_OUTPUT("DMS2HMI_Warning_First", MAPS::Integer32, nullptr, nullptr, 1)
    MAPS_OUTPUT("DMS2HMI_Warning_Second", MAPS::Integer32, nullptr, nullptr, 1)
    MAPS_OUTPUT("DMS2CAV_LockOut", MAPS::Integer32, nullptr, nullptr, 1)
MAPS_END_OUTPUTS_DEFINITION

MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSDriverMonitoring)
MAPS_END_PROPERTIES_DEFINITION

MAPS_BEGIN_ACTIONS_DEFINITION(MAPSDriverMonitoring)
MAPS_END_ACTIONS_DEFINITION

MAPS_COMPONENT_DEFINITION(MAPSDriverMonitoring, "DMS_System", "1.0.0", 128,
                          MAPS::Threaded, MAPS::Threaded,
                          4, // Nb of inputs
                          3, // Nb of outputs
                          0,
                          0)

void MAPSDriverMonitoring::Birth()
{
    m_inputReader = MAPS::MakeInputReader::PeriodicSampling(
        this,
        50000, // 50ms interval
        MAPS::InputReaderOption::PeriodicSampling::SamplingBehavior::WaitForAllInputs,
        MAPS::MakeArray(&Input("DMSIP_DrvrAttnStatAuth"),
                       &Input("StrgWhlTchSnsHndsOnStat"), &Input("HMI2DMS_ActivateSignal"),
                       &Input("ActETRS")),
        &MAPSDriverMonitoring::ProcessData
    );

    m_handsOffStartTime = 0;
    m_handsOffFirstWarningIssued = false;
    m_activeWarning = 0;
    m_attentionScore = 0;
    m_attentiveTimestamps.clear();
    m_lockoutStartTime = 0;
    m_lockoutActive = false;
}

void MAPSDriverMonitoring::Core()
{
    m_inputReader->Read();
}

void MAPSDriverMonitoring::Death()
{
    m_inputReader.reset();
}

void MAPSDriverMonitoring::ProcessData(MAPSTimestamp ts, MAPS::InputElt<int64_t> inElt1,
                                     MAPS::InputElt<int64_t> inElt2, MAPS::InputElt<int64_t> inElt3,
                                     MAPS::InputElt<int64_t> inElt4)
{
    int64_t attnStat       = inElt1.Data();
    int64_t handsOnStat    = inElt2.Data();
    int64_t activateSignal = inElt3.Data();
    int64_t gearState      = inElt4.Data();

    MAPS::OutputGuard<int32_t> warnFirstGuard{this, Output("DMS2HMI_Warning_First")};
    MAPS::OutputGuard<int32_t> warnSecondGuard{this, Output("DMS2HMI_Warning_Second")};
    MAPS::OutputGuard<int32_t> lockoutGuard{this, Output("DMS2CAV_LockOut")};

    warnFirstGuard.Data(0) = 0;
    warnSecondGuard.Data(0) = 0;
    lockoutGuard.Data(0) = 0;

    warnFirstGuard.VectorSize() = 1;
    warnSecondGuard.VectorSize() = 1;
    lockoutGuard.VectorSize() = 1;

    if (activateSignal == 1 && gearState == 4) {
        int handsOffWarningLevel = 0;
        int inattentionWarningLevel = 0;

        // Attention score tracking
        if (attnStat == 1) {
            m_attentiveTimestamps.push_back(ts);
            m_attentionScore -= 2;
            if (m_attentionScore < 0) m_attentionScore = 0;
        } else {
            m_attentionScore += 1;
        }

        while (!m_attentiveTimestamps.empty() && (ts - m_attentiveTimestamps.front()) > 7000000) {
            m_attentiveTimestamps.pop_front();
        }

        if (m_attentiveTimestamps.size() >= 150) {
            m_attentionScore = 0;
        }

        // Inattention warnings: first at 5s (score 100), second 15s after first (score 400)
        if (m_attentionScore >= 100 && m_activeWarning != 2) {
            inattentionWarningLevel = 1;
        }
        if (m_attentionScore >= 400) {
            inattentionWarningLevel = 2;
        }

        // Hands-off warnings: first at 5s, second at 15s total (10s after first)
        if (handsOnStat == 0 && m_activeWarning != 2) {
            if (m_handsOffStartTime == 0) {
                m_handsOffStartTime = ts;
            }
            double elapsedHandsOff = (ts - m_handsOffStartTime) / 1000000.0;

            if (elapsedHandsOff >= 15.0) {
                handsOffWarningLevel = 2;
            } else if (elapsedHandsOff >= 5.0) {
                handsOffWarningLevel = 1;
                m_handsOffFirstWarningIssued = true;
            }
        } else if (handsOnStat != 0) {
            m_handsOffStartTime = 0;
            m_handsOffFirstWarningIssued = false;
        }

        // Warning state machine
        if (m_activeWarning == 2) {
            if (handsOnStat != 0 && attnStat == 1) {
                m_activeWarning = 0;
                m_handsOffStartTime = 0;
                m_attentionScore = 0;
                m_attentiveTimestamps.clear();
            } else {
                warnSecondGuard.Data(0) = 1;
            }
        } else if (m_activeWarning == 1) {
            if (handsOffWarningLevel == 2 || inattentionWarningLevel == 2) {
                m_activeWarning = 2;
                warnSecondGuard.Data(0) = 1;
            } else if (handsOnStat != 0 && attnStat == 1) {
                m_activeWarning = 0;
                m_handsOffStartTime = 0;
                m_attentionScore = 0;
                m_attentiveTimestamps.clear();
            } else {
                warnFirstGuard.Data(0) = 1;
            }
        } else {
            if (handsOffWarningLevel == 2 || inattentionWarningLevel == 2) {
                m_activeWarning = 2;
                warnSecondGuard.Data(0) = 1;
            } else if (handsOffWarningLevel == 1 || inattentionWarningLevel == 1) {
                m_activeWarning = 1;
                warnFirstGuard.Data(0) = 1;
            }
        }

        // Start lockout on first entry into second warning; hold for 30s
        if (m_activeWarning == 2 && !m_lockoutActive) {
            m_lockoutActive = true;
            m_lockoutStartTime = ts;
        }
        if (m_lockoutActive && ((ts - m_lockoutStartTime) / 1000000.0 >= 30.0)) {
            m_lockoutActive = false;
        }
        lockoutGuard.Data(0) = m_lockoutActive ? 1 : 0;

    } else {
        // Reset all state when inactive or not in drive
        m_handsOffStartTime = 0;
        m_handsOffFirstWarningIssued = false;
        m_activeWarning = 0;
        m_attentionScore = 0;
        m_attentiveTimestamps.clear();
        m_lockoutActive = false;
        m_lockoutStartTime = 0;
    }

    warnFirstGuard.Timestamp() = ts;
    warnSecondGuard.Timestamp() = ts;
    lockoutGuard.Timestamp() = ts;
}
