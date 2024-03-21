#pragma once
#include <Arduino.h>
#include <esp_dsp.h>


class Buffer{

    double * buf;
    double * t_buf; // time in seconds
    int n;
    bool is_full = false;
    int curr_i = 0;

    // Things to calculate gradient with
    double t_avg = 0;
    double y_avg = 0;
    // double t_t = 0;
    // double t_y = 0;

    //things for gaussian filter
    double * taps;
    void calculate_taps();

    // stores the gradient
    double slope;

    public:
    Buffer(int buf_size);

    void insert(double t, double y);

    void clear();

    double getSlope();

    double getAverage();

    double getFiltered();

};