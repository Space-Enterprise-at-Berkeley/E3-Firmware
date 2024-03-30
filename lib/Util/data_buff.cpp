#include <data_buff.h>
#include <Arduino.h>

Buffer::Buffer(int buf_size){
    buf = (double *)malloc(sizeof(double) * buf_size);
    t_buf = (double *)malloc(sizeof(double) * buf_size);
    n = buf_size;
    taps = (double *)malloc(sizeof(double) * buf_size);
    calculate_taps();
    clear();
}

void Buffer::insert(double t, double y) {
    // inserts new data point at position pointed by index
    // and recalculates slope

    // recalculate slope
    t_avg += (t - t_buf[curr_i])/double(n);
    y_avg += (y - buf[curr_i])/double(n);
    
    // update buffers
    buf[curr_i] = y;
    t_buf[curr_i] = t;

    double num = 0;
    double denom = 0;
    for (int i = 0; i<n; i++) {
        num += (t_buf[i]-t_avg) * (buf[i]-y_avg);
        denom += (t_buf[i]-t_avg) * (t_buf[i]-t_avg);
        // t_avg += t_buf[i]/double(n);
        // y_avg += buf[i]/double(n);
        // t_t += t_buf[i] * t_buf[i];
        // t_y += t_buf[i] * buf[i];
    
    }
    // slope = (t_y - n*t_avg*y_avg)/(t_t - n*t_avg*t_avg);
    slope = num/denom;

    // increment indices
    curr_i++;
    curr_i = curr_i % n;
    if (curr_i == 0) {
        is_full = true;
    }
}

void Buffer::clear() {
    curr_i = 0;
    is_full = false;
    t_avg = 0;
    y_avg = 0;
    // t_t = 0;
    // t_y = 0;
    for (int i = 0; i<n; i++) {
        buf[i] = 0;
        t_buf[i] = 0;
    }
}

double Buffer::getSlope() {
    // returns 0 if buffer hasn't been filled
    return is_full ? slope : 0;
}

double Buffer::getAverage() {
    if (!is_full && curr_i == 0) { // buffer is completely empty
        return 0;
    }
    return is_full ? y_avg : (y_avg * n) / curr_i;
}
double Buffer::getFiltered() {
    if (!is_full) {
        return 0;
    } else {
        double filtered = 0;
        for (int i = curr_i; i<curr_i+n; i++) {
            filtered += buf[i % n] * taps[i - curr_i];
        }
        return filtered;
    }
}

void Buffer::calculate_taps() {
    double stdev = (float)n / 1.5; //2.35; //fwhm of ~0.5
    double sum = 0;
    for (int i = 0; i<n; i++) {
        taps[i] = exp(-0.5 * pow((i - (n/2))/stdev, 2));
        sum += taps[i];
    }
    for (int i = 0; i<n; i++) {
        taps[i] /= sum;
    }
}

