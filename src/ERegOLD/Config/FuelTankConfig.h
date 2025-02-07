//Fuel EReg Config 
#pragma once
#include "../FlowProfiles.h"

namespace Config {

    #define MAX_ANGLE 650                                                
    #define MIN_ANGLE 0
    #define ANTIWINDUP_RANGE_LOWER 150
    #define ANTIWINDUP_RANGE_UPPER 650

    #define OUTER_BUFFER_SIZE 3

    // Controller Constants

    // const double p_outer_nominal = 1.8, i_outer_nominal = 0.30e-6, d_outer_nominal = 0; 
    // from cryo flow march 4, ~8:30PM - ~1700PSI to 300PSI. Steady state offset of -20PSI. increased constants by 1.2x.

    // const double p_outer_nominal = 2.2, i_outer_nominal = 0.4e-6, d_outer_nominal = 0; // nominal is 4000 -> 500 psi flow
    const double initial_p_outer_nominal = 3.3, initial_i_outer_nominal = 0.6e-6, initial_d_outer_nominal = 0; // nominal is 4000 -> 500 psi flow //for Mar 11, Dev flow #1

    // const double p_inner = 6, i_inner = 3.5e-6, d_inner = 0.10;
    const double initial_p_inner = 3, initial_i_inner = 5e-6, initial_d_inner = 0;


    // Flow Parameters
    const float initial_pressureSetpoint = 530; // hotfire 7 was 495
    const unsigned long rampStart = 0.7 * pressureSetpoint; // psi
    const unsigned long loxLead = 0; //time in microseconds, used to be 105ms

    // Diagnostic configs
    const int servoTestPoints = 5;
    const float servoTravelInterval = 100; // encoder counts
    const unsigned long servoSettleTime =  200UL * 1000UL; // micros
    const float stopDiagnosticPressureThresh = 150; // diagnostic terminates if either tank exceeds this
    const float diagnosticSpeed = 200;
}
