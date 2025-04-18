#include "Config.h"

namespace Config {
    
    unsigned long flowDuration = 0;
    float pressureSetpoint, boiloffDrop, boiloffEnd, p_outer_nominal, i_outer_nominal, d_outer_nominal, p_inner, i_inner, d_inner, pressurizationCutoff;

    unsigned long rampStart;

    void setFlowDuration(unsigned long duration) {
        flowDuration = duration;
    }

    unsigned long getFlowDuration() {
        return flowDuration;
    }

    void init() {
        // try getting vars from EEPROM, if not found, use values from Config.h
        //configInitialized = true;
        EEPROM.begin(17*sizeof(float));
        EEPROM.get(8*sizeof(float), pressureSetpoint);
        if (isnan(pressureSetpoint)){
            pressureSetpoint = initial_pressureSetpoint;
        }
        rampStart = 0.7 * pressureSetpoint; 
        pressurizationCutoff = pressureSetpoint * 0.99;
        EEPROM.get(9*sizeof(float), boiloffDrop);
        if (isnan(boiloffDrop)){
            boiloffDrop = initial_boiloffDrop;
        }
        EEPROM.get(10*sizeof(float), boiloffEnd);
        if (isnan(boiloffEnd)){
            boiloffEnd = initial_boiloffEnd;
        }
        EEPROM.get(11*sizeof(float), p_outer_nominal);
        if (isnan(p_outer_nominal)){
            p_outer_nominal = initial_p_outer_nominal;
        }
        EEPROM.get(12*sizeof(float), i_outer_nominal);
        if (isnan(i_outer_nominal)){
            i_outer_nominal = initial_i_outer_nominal;
        }
        EEPROM.get(13*sizeof(float), d_outer_nominal);
        if (isnan(d_outer_nominal)){
            d_outer_nominal = initial_d_outer_nominal;
        }
        EEPROM.get(14*sizeof(float), p_inner);
        if (isnan(p_inner)){
            p_inner = initial_p_inner;
        }
        EEPROM.get(15*sizeof(float), i_inner);
        if (isnan(i_inner)){
            i_inner = initial_i_inner;
        }
        EEPROM.get(16*sizeof(float), d_inner);
        if (isnan(d_inner)){
            d_inner = initial_d_inner;
        }
        EEPROM.end();
    }

    void setPressureSetpoint(float setpoint) {
        pressureSetpoint = setpoint;
        pressurizationCutoff = pressureSetpoint * 0.99;
        rampStart = 0.7 * pressureSetpoint; // psi
        EEPROM.begin(17*sizeof(float));
        EEPROM.put(8*sizeof(float), pressureSetpoint);
        EEPROM.end();
    }

    void setBoiloffDrop(float drop) {
        boiloffDrop = drop / (1000000); //psi per second
        EEPROM.begin(17*sizeof(float));
        EEPROM.put(9*sizeof(float), boiloffDrop);
        EEPROM.end();
    }

    void setBoiloffEnd(float end) {
        boiloffEnd = end;
        EEPROM.begin(17*sizeof(float));
        EEPROM.put(10*sizeof(float), boiloffEnd);
        EEPROM.end();
    }

    void setPOuter(float p) {
        p_outer_nominal = p;
        EEPROM.begin(17*sizeof(float));
        EEPROM.put(11*sizeof(float), p_outer_nominal);
        EEPROM.end();
    }

    void setIOuter(float i) {
        i_outer_nominal = i;
        EEPROM.begin(17*sizeof(float));
        EEPROM.put(12*sizeof(float), i_outer_nominal);
        EEPROM.end();
    }

    void setDOuter(float d) {
        d_outer_nominal = d;
        EEPROM.begin(17*sizeof(float));
        EEPROM.put(13*sizeof(float), d_outer_nominal);
        EEPROM.end();
    }

    void setPInner(float p) {
        p_inner = p;
        EEPROM.begin(17*sizeof(float));
        EEPROM.put(14*sizeof(float), p_inner);
        EEPROM.end();
    }

    void setIInner(float i) {
        i_inner = i;
        EEPROM.begin(17*sizeof(float));
        EEPROM.put(15*sizeof(float), i_inner);
        EEPROM.end();
    }

    void setDInner(float d) {
        d_inner = d;
        EEPROM.begin(17*sizeof(float));
        EEPROM.put(16*sizeof(float), d_inner);
        EEPROM.end();
    }
}