//LOX EReg Config 
#pragma once
#include "../FlowProfiles.h"

namespace Config {

    #define MAX_ANGLE 850             // for IPA ereg, true max is ~850                                    
    #define MIN_ANGLE 0               // for IPA ereg, true min is ~40 (for full close limit switch to be hit)
    #define ANTIWINDUP_RANGE_LOWER 150
    #define ANTIWINDUP_RANGE_UPPER 850

    #define OUTER_BUFFER_SIZE 3

    // Controller Constants
    // const double p_outer_nominal = 1.8, i_outer_nominal = 0.30e-6, d_outer_nominal = 0.24; // nominal is 4000 -> 500 psi flow
    // const double p_outer_nominal = 2.2, i_outer_nominal = 0.4e-6, d_outer_nominal = 0; // nominal is 4000 -> 500 psi flow
    const double p_outer_nominal = 3.3, i_outer_nominal = 0.9e-6, d_outer_nominal = 0; // nominal is 4000 -> 500 psi flow //for Mar 11, Dev flow #1
    // const double p_inner = 6, i_inner = 3.5e-6, d_inner = 0.10;
    const double p_inner = 3, i_inner = 5e-6, d_inner = 0;

    // Flow Parameters
    const float pressureSetpoint = 465;
    const unsigned long rampStart = 0.7 * pressureSetpoint; // psi
    const unsigned long loxLead = 0; //time in microseconds
    const float boiloffDrop = 10.33 / (1000000) ; //psi per second
    const float boiloffEnd = 372;

    // Diagnostic configs
    const int servoTestPoints = 5;
    const float servoTravelInterval = 100; // encoder counts
    const unsigned long servoSettleTime =  200UL * 1000UL; // micros
    const float stopDiagnosticPressureThresh = 150; // diagnostic terminates if either tank exceeds this
    const float diagnosticSpeed = 200; // speed at which to run open loop motors
}
