///////////////////////////////////////////////////////////////////////////////
//    This file is part of RTMaps                                             //
//    Copyright (c) Intempora S.A. All rights reserved.                       //
///////////////////////////////////////////////////////////////////////////////

#pragma once
#include <maps.hpp>
#include <maps/input_reader/maps_input_reader.hpp>
#include <memory> // For std::unique_ptr
#include <deque>  // For std::deque

class MAPSDriverMonitoring : public MAPSComponent
{
    MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSDriverMonitoring)
private:
    std::unique_ptr<MAPS::InputReader> m_inputReader;
    MAPSTimestamp m_handsOffStartTime;         // Timestamp when hands-off started
    bool m_handsOffFirstWarningIssued;         // Flag for first hands-off warning
    int m_initialPhase;                       // Counter for initial 5-second phase
    int m_noDriverInitialCount;               // Counter for consecutive "no driver" states in initial phase
    bool m_initialNoDriverSent;               // Flag to ensure initial "No Driver" message is sent once
    int m_activeWarning;                      // Tracks active warning: 0 (none), 1 (first), 2 (second)
    int m_attentionScore;                     // Accumulated inattention score
    std::deque<MAPSTimestamp> m_attentiveTimestamps; // Timestamps when driver was attentive

private:
    // Input and Output definitions should NOT be here

public:
    void ProcessData(MAPSTimestamp ts, MAPS::InputElt<int64_t> inElt1, MAPS::InputElt<double> inElt2, 
                     MAPS::InputElt<int64_t> inElt3, MAPS::InputElt<int64_t> inElt4, MAPS::InputElt<int64_t> inElt5);

    // Birth, Core, and Death methods are declared in the standard header code
};
