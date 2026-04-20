#include <algorithm>
#include "maps_DriverMonitoring.h"

// inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSDriverMonitoring)
    MAPS_INPUT("DMSIP_DrvrAttnStatAuth", MAPS::FilterInteger64, MAPS::SamplingReader)
    MAPS_INPUT("DMSIP_DrvrAttnStatPrdAuth", MAPS::FilterFloat64, MAPS::SamplingReader)
    MAPS_INPUT("StrgWhlTchSnsHndsOnStat", MAPS::FilterInteger64, MAPS::SamplingReader)
    MAPS_INPUT("HMI2DMS_ActivateSignal", MAPS::FilterInteger64, MAPS::SamplingReader)
    MAPS_INPUT("ActETRS", MAPS::FilterInteger64, MAPS::SamplingReader)
MAPS_END_INPUTS_DEFINITION

// outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSDriverMonitoring)
    MAPS_OUTPUT("DMS2HMI_Warning_First", MAPS::Integer32, nullptr, nullptr, 1)
    MAPS_OUTPUT("DMS2HMI_Warning_Second", MAPS::Integer32, nullptr, nullptr, 1)
    MAPS_OUTPUT("DMS2HMI_NoDriverDetected", MAPS::Integer32, nullptr, nullptr, 1)
    MAPS_OUTPUT("NoDriverDMS2HMI", MAPS::Integer32, nullptr, nullptr, 1)
MAPS_END_OUTPUTS_DEFINITION

MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSDriverMonitoring)
MAPS_END_PROPERTIES_DEFINITION

MAPS_BEGIN_ACTIONS_DEFINITION(MAPSDriverMonitoring)
MAPS_END_ACTIONS_DEFINITION

MAPS_COMPONENT_DEFINITION(MAPSDriverMonitoring, "DMS_System", "1.0.0", 128,
                          MAPS::Threaded, MAPS::Threaded,
                          5, // Nb of inputs
                          4, // Nb of outputs
                          0,
                          0)

void MAPSDriverMonitoring::Birth()
{
    m_inputReader = MAPS::MakeInputReader::PeriodicSampling(
        this,
        50000, // 50ms interval
        MAPS::InputReaderOption::PeriodicSampling::SamplingBehavior::WaitForAllInputs,
        MAPS::MakeArray(&Input("DMSIP_DrvrAttnStatAuth"), &Input("DMSIP_DrvrAttnStatPrdAuth"), 
                       &Input("StrgWhlTchSnsHndsOnStat"), &Input("HMI2DMS_ActivateSignal"), &Input("ActETRS")),
        &MAPSDriverMonitoring::ProcessData
    );

    m_handsOffStartTime = 0;
    m_handsOffFirstWarningIssued = false;
    m_initialPhase = 0;
    m_noDriverInitialCount = 0;
    m_initialNoDriverSent = false;
    m_activeWarning = 0;
    m_attentionScore = 0;
    m_attentiveTimestamps.clear();
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
                                     MAPS::InputElt<double> inElt2, MAPS::InputElt<int64_t> inElt3, 
                                     MAPS::InputElt<int64_t> inElt4, MAPS::InputElt<int64_t> inElt5)
{
    // input data
    int64_t attnStat = inElt1.Data();
    double attnPrd = inElt2.Data();
    int64_t handsOnStat = inElt3.Data();
    int64_t activateSignal = inElt4.Data();
    int64_t gearState = inElt5.Data();

    // output guards
    MAPS::OutputGuard<int32_t> warnFirstGuard{this, Output("DMS2HMI_Warning_First")};
    MAPS::OutputGuard<int32_t> warnSecondGuard{this, Output("DMS2HMI_Warning_Second")};
    MAPS::OutputGuard<int32_t> noDriverGuard{this, Output("DMS2HMI_NoDriverDetected")};
    MAPS::OutputGuard<int32_t> noDriverDMS2HMIGuard{this, Output("NoDriverDMS2HMI")};

    // Initialize outputs to 0
    warnFirstGuard.Data(0) = 0;
    warnSecondGuard.Data(0) = 0;
    noDriverGuard.Data(0) = 0;
    noDriverDMS2HMIGuard.Data(0) = 0;

    warnFirstGuard.VectorSize() = static_cast<int>(1);
    warnSecondGuard.VectorSize() = static_cast<int>(1);
    noDriverGuard.VectorSize() = static_cast<int>(1);
    noDriverDMS2HMIGuard.VectorSize() = static_cast<int>(1);

    // Check if system is active and in drive
    if (activateSignal == 1 && gearState == 4) {
        // Initial driver detection phase (first 5 seconds)
        if (m_initialPhase < 100) { // 100 cycles = 5 seconds
            if (attnStat != 1) {
                m_noDriverInitialCount++;
                if (m_noDriverInitialCount >= 100 && !m_initialNoDriverSent) {
                    noDriverDMS2HMIGuard.Data(0) = 1;
                    m_initialNoDriverSent = true;
                    m_initialPhase = 100;
                    return;
                }
            } else {
                m_noDriverInitialCount = 0;
            }
            m_initialPhase++;
            return;
        }

        // Track warning states
        int handsOffWarningLevel = 0;
        int inattentionWarningLevel = 0;

        // Attention tracking (from first code)
        if (attnStat == 1) {
            m_attentiveTimestamps.push_back(ts);
            m_attentionScore -= 2;
            if (m_attentionScore < 0) {
                m_attentionScore = 0;
            }
        } else {
            m_attentionScore += 1;
        }

        // Remove timestamps older than 7 seconds
        while (!m_attentiveTimestamps.empty() && (ts - m_attentiveTimestamps.front()) > 7000000) {
            m_attentiveTimestamps.pop_front();
        }

        // Check for 5 seconds of attention in 7-second window
        if (m_attentiveTimestamps.size() >= 150) {
            m_attentionScore = 0;
        }

        // Set inattention warning levels
        if (m_attentionScore >= 80 && m_activeWarning != 2) { // 5 seconds
            inattentionWarningLevel = 1;
        }
        if (m_attentionScore >= 280) { // 14 seconds
            inattentionWarningLevel = 2;
        }

        // Hands-off-wheel logic
        if (handsOnStat == 0 && m_activeWarning != 2) {
            if (m_handsOffStartTime == 0) {
                m_handsOffStartTime = ts;
            }
            double elapsedHandsOff = (ts - m_handsOffStartTime) / 1000000.0;

            if (elapsedHandsOff >= 9.0) {
                handsOffWarningLevel = 2;
                m_handsOffFirstWarningIssued = false;
            } else if (elapsedHandsOff >= 4.0 && !m_handsOffFirstWarningIssued) {
                handsOffWarningLevel = 1;
                m_handsOffFirstWarningIssued = true;
            }
        } else if (handsOnStat != 0) {
            m_handsOffStartTime = 0;
            m_handsOffFirstWarningIssued = false;
        }

        // Centralized warning logic (from second code)
        if (m_activeWarning == 2) {
            if (handsOnStat != 0 && attnStat == 1) {
                m_activeWarning = 0;
                m_handsOffStartTime = 0;
                m_attentionScore = 0;
                m_attentiveTimestamps.clear();
            } else {
                warnSecondGuard.Data(0) = 1;
                warnFirstGuard.Data(0) = 0;
            }
        } else if (m_activeWarning == 1) {
            if (handsOffWarningLevel == 2 || inattentionWarningLevel == 2) {
                m_activeWarning = 2;
                warnSecondGuard.Data(0) = 1;
                warnFirstGuard.Data(0) = 0;
            } else if (handsOnStat != 0 && attnStat == 1) {
                m_activeWarning = 0;
                m_handsOffStartTime = 0;
                m_attentionScore = 0;
                m_attentiveTimestamps.clear();
            } else {
                warnFirstGuard.Data(0) = 1;
                warnSecondGuard.Data(0) = 0;
            }
        } else {
            if (handsOffWarningLevel == 2 || inattentionWarningLevel == 2) {
                m_activeWarning = 2;
                warnSecondGuard.Data(0) = 1;
                warnFirstGuard.Data(0) = 0;
            } else if (handsOffWarningLevel == 1 || inattentionWarningLevel == 1) {
                m_activeWarning = 1;
                warnFirstGuard.Data(0) = 1;
                warnSecondGuard.Data(0) = 0;
            }
        }

        // No driver logic
        if (attnStat == 10 && attnPrd >= 5.0) {
            noDriverGuard.Data(0) = 1;
        }
    } else {
        // Reset all state when inactive or not in drive
        m_handsOffStartTime = 0;
        m_handsOffFirstWarningIssued = false;
        m_initialPhase = 0;
        m_noDriverInitialCount = 0;
        m_initialNoDriverSent = false;
        m_activeWarning = 0;
        m_attentionScore = 0;
        m_attentiveTimestamps.clear();
    }

    // Set timestamps for outputs
    warnFirstGuard.Timestamp() = ts;
    warnSecondGuard.Timestamp() = ts;
    noDriverGuard.Timestamp() = ts;
    noDriverDMS2HMIGuard.Timestamp() = ts;
}
