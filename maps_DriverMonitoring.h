///////////////////////////////////////////////////////////////////////////////
//    This file is part of RTMaps                                             //
//    Copyright (c) Intempora S.A. All rights reserved.                       //
///////////////////////////////////////////////////////////////////////////////

#pragma once
#include <maps.hpp>
#include <maps/input_reader/maps_input_reader.hpp>
#include <memory>
#include <deque>

class MAPSDriverMonitoring : public MAPSComponent
{
    MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSDriverMonitoring)
private:
    std::unique_ptr<MAPS::InputReader> m_inputReader;
    MAPSTimestamp m_handsOffStartTime;
    bool m_handsOffFirstWarningIssued;
    int m_activeWarning;
    int m_attentionScore;
    std::deque<MAPSTimestamp> m_attentiveTimestamps;
    MAPSTimestamp m_lockoutStartTime;
    bool m_lockoutActive;

public:
    void ProcessData(MAPSTimestamp ts, MAPS::InputElt<int64_t> inElt1,
                     MAPS::InputElt<int64_t> inElt2, MAPS::InputElt<int64_t> inElt3,
                     MAPS::InputElt<int64_t> inElt4);
};
