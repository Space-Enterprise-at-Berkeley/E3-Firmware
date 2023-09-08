#define STATE_SIZE 3
#define OBS_SIZE 3

#include <ArduinoEigenDense.h>

using namespace Eigen;

namespace Config{
    const Matrix<double, STATE_SIZE, STATE_SIZE> transition { 
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    const Matrix<double, STATE_SIZE, STATE_SIZE> transition_dt { 
        {0, 1, 0},
        {0, 0, 1},
        {0, 0, 0}
    };
    const Matrix<double, STATE_SIZE, STATE_SIZE> state_noise {
        {0.001, 0, 0},
        {0, 0.9, 0},
        {0, 0, 0.9},
    };
    const Matrix<double, OBS_SIZE, STATE_SIZE> obs {
        {1, 0, 0},
        {1, 0, 0},
        {0 ,0 ,1}
    };
    const Matrix<double, OBS_SIZE, OBS_SIZE> obs_noise {
        {10, 0 ,0},
        {0, 10000, 0},
        {0, 0, 0.01}
    };
}


using namespace Eigen;

class KalmanFilter{
    /*
    Kalman filter for apogee detection
    Observation and state space dimensions are fixed through configs and defs
    Dynamics, measurement, and noise matrix values can be modified per filter instance
    Variables:
    A = Transition matrix (STATE_SIZE x STATE_SIZE)
    Q = State noise covariance (STATE_SIZE x STATE_SIZE)
    C = Measurement matrix (OBS_SIZE x STATE_SIZE)
    R = Measurement noise covariance (OBS_SIZE x OBS_SIZE)
    */

    private:
    Matrix<double, STATE_SIZE, STATE_SIZE> A;
    Matrix<double, STATE_SIZE, STATE_SIZE> A_dt;
    Matrix<double, STATE_SIZE, STATE_SIZE> Q;
    Matrix<double, OBS_SIZE, STATE_SIZE> C;
    Matrix<double, OBS_SIZE, OBS_SIZE> R;

    Vector<double, STATE_SIZE> state;
    Matrix<double, STATE_SIZE, STATE_SIZE> P;
    Matrix<double, STATE_SIZE, STATE_SIZE> prev_P;

    Vector<double, OBS_SIZE> measurement;
    unsigned long last_update_micros;

    const Matrix<double, STATE_SIZE, STATE_SIZE> I = Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();

    void predict(double dt);
    void _update(Vector<double, OBS_SIZE>& measurement, double dt);
    Vector<double, STATE_SIZE>& state_estimate();

    public:
    KalmanFilter(
        const Matrix<double, STATE_SIZE, STATE_SIZE>& transition,
        const Matrix<double, STATE_SIZE, STATE_SIZE>& transition_dt,
        const Matrix<double, STATE_SIZE, STATE_SIZE>& state_noise,
        const Matrix<double, OBS_SIZE, STATE_SIZE>& obs,
        const Matrix<double, OBS_SIZE, OBS_SIZE>& obs_noise
    ); 

    void reset();
    void update(double gps, double barometer, double accel);
    double get_altitude();
    double get_velocity();
    double get_acceleration();
};


KalmanFilter::KalmanFilter(
    const Matrix<double, STATE_SIZE, STATE_SIZE>& transition,
    const Matrix<double, STATE_SIZE, STATE_SIZE>& transition_dt,
    const Matrix<double, STATE_SIZE, STATE_SIZE>& state_noise,
    const Matrix<double, OBS_SIZE, STATE_SIZE>& obs,
    const Matrix<double, OBS_SIZE, OBS_SIZE>& obs_noise
) {
    A = transition;
    A_dt = transition_dt;
    Q = state_noise;
    C = obs;
    R = obs_noise;

    reset();
}

void KalmanFilter::reset() {
    state = Vector<double, STATE_SIZE>::Zero();
    prev_P = Matrix<double, STATE_SIZE, STATE_SIZE>::Zero();
    P = Matrix<double, STATE_SIZE, STATE_SIZE>::Zero();
    measurement = Vector<double, OBS_SIZE>::Zero();
}

void KalmanFilter::predict(double dt) {
    prev_P = P; // I think this deepcopies P, but not rly sure tbh ¯\_(ツ)_/¯
    Matrix<double, STATE_SIZE, STATE_SIZE> A_current = (A + A_dt * dt);
    state = A_current * state;
    P = A_current * P * A_current.transpose() + Q * dt;
}

void KalmanFilter::_update(Vector<double, OBS_SIZE>& measurement, double dt) {
    predict(dt);
    Matrix<double, STATE_SIZE, STATE_SIZE> K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    P = (I - K * C) * P;
    state += K * (measurement - C * state);
}

Vector<double, STATE_SIZE>& KalmanFilter::state_estimate() {
    return state;
}

void KalmanFilter::update(double gps, double barometer, double accel) {
    /*
    Updates Kalman filter with latest measurement. This function should only be called when we have new updates for ALL 3 sensors
    @param gps: gps height in m
    @param barometer: barometer altitude in m
    @param accel: accelerometer reading (in vertical direction) in g
    */

    measurement(0) = gps;
    measurement(1) = barometer;
    measurement(2) = accel;
   
    unsigned long t1 = micros();
    double dt = (t1 - last_update_micros)/1e6;
    last_update_micros = t1;
    _update(measurement, dt);
    
}

double KalmanFilter::get_altitude() {
    /*
    Returns current state estimate of altitude, as of previous update
    @return altitude in m
    */
    return state(0);
}
double KalmanFilter::get_velocity() {
    /*
    Returns current state estimate of vertical velocity, as of previous update
    @return velocity in m/s
    */
    return state(1);
}
double KalmanFilter::get_acceleration() {
    /*
    Returns current state estimate of vertical acceleration, as of previous update
    @return acceleration in m/s^2
    */
    return state(2);
}


namespace FlightStatus { 
    
    KalmanFilter filter1(Config::transition, Config::transition_dt, Config::state_noise, Config::obs, Config::obs_noise);

    int launched = 0;
    int burnout = 0;
    int apogee = 0;
    int main_parachute = 0;
    float prev_altitude = 0;
    int deploy_vel = 0;

    uint32_t flightDataUpdate = 100 * 1000;

    uint32_t updateFlight() {

        // TODO: Read IMUs, barometers, GPS, and send packets

        baro_altitude = get_barometer_altitude();
        acceleration = get_acceleration_y();

        if (gps_altitude < 50) {
            gps_altitude = baro_altitude;
        }

        if (7.8 < acceleration & 11.8 > acceleration) {
            acceleration = 0;
        }
        if (acceleration < 0) {
            acceleration = 0;
        }

        filter1.update(gps_altitude, baro_altitude, acceleration);

        double alt = filter1.get_altitude();
        double vel = filter1.get_velocity();
        double accl = filter1.get_acceleration();

        if (vel > 3 & accl > 15 & launched == 0) {
            launched = 300;
        }

        if (launched > 0 & accl < 5 & burnout == 0) {
            burnout = 200;
        }

        if (burnout > 0 & alt < prev_altitude & apogee == 0 & vel < deploy_vel) {
            apogee = 100;
            deploy_drogue();
        }

        if (apogee > 0 & alt < main_altitude & main_parachute == 0) {
            main_parachute = alt;
            deploy_main();
        }

    	// todo: set up relevant packets 
        // Comms::packetAddFloat(); 

        return flightDataUpdate;
    }


}