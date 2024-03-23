//Fuel Injector EReg Config 
#pragma once
#include "../FlowProfiles.h"

namespace Config {

    #define MAX_ANGLE 900                                                    
    #define MIN_ANGLE 0
    #define ANTIWINDUP_RANGE_LOWER 250
    #define ANTIWINDUP_RANGE_UPPER 900

    #define OUTER_BUFFER_SIZE 2

    // Controller Constants
    const double p_outer_nominal = 0.5, i_outer_nominal = 0.6e-6, d_outer_nominal = 0.0;
    const double p_inner = 6, i_inner = 4.5e-6, d_inner = 0.15;

    // Flow Parameters
    const float pressureSetpoint = 460;
    const unsigned long loxLead = 110UL * 1000UL; //time in microseconds

    // Diagnostic configs
    const int servoTestPoints = 9;
    const float servoTravelInterval = 100; // encoder counts
    const unsigned long servoSettleTime =  500UL * 1000UL; // micros
    const float stopDiagnosticPressureThresh = 700; // diagnostic terminates if either tank exceeds this
    const float diagnosticSpeed = 0;
}