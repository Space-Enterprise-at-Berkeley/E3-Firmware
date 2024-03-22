#include "TimeUtil.h"

namespace TimeUtil {
    /**
     * Computes time interval between 2 times, while handling overflow issues
     * Here we make 2 assumptions:
     * - Time interval does not exceed 4e9 us (~70 min), otherwise interval % INT_MAX is returned
     * - earlierTime happens at an earlier time than laterTime
     * @param earlierTime timestamp of 1st (in chronological order) event
     * @param laterTime timestamp of 2nd (in chronological order) event
     * @return elapsed time between 1st and 2nd event
     */
    unsigned long timeInterval(unsigned long earlierTime, unsigned long laterTime) {
        return laterTime - earlierTime;
    }
}