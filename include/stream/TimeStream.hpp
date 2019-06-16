#ifndef TIME_STREAM_H
#define TIME_STREAM_H

#include <sys/time.h>

// Since detections are made offline, we simulate timestamp used in the
// kinect frames at 30 Hz

class TimeStream {
    public:
        TimeStream(float freq = 30.f);
        ~TimeStream();

        timeval get_next();

        timeval get_current();

        bool advance_step();

    private:
        int acquisition_frequency;
        double step_in_useconds;

        timeval current_timestamp;
};

#endif //TIME_STREAM_H
