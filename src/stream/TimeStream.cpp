#include "stream/TimeStream.hpp"

TimeStream::TimeStream(float freq){
    acquisition_frequency = freq;

    step_in_useconds = (1/freq)*1000000;

    current_timestamp.tv_sec = 0;
    current_timestamp.tv_usec = 0;
}

TimeStream::~TimeStream(){

}

timeval TimeStream::get_next(){
    advance_step();
    return current_timestamp;
}

timeval TimeStream::get_current(){
    return current_timestamp;
}

bool TimeStream::advance_step(){
    // get time in usec
    double time_in_usec = (current_timestamp.tv_sec * 1000000) +
                          (current_timestamp.tv_usec);
    time_in_usec += step_in_useconds;
    int time_sec = time_in_usec/1000000;
    int time_usec = time_in_usec - time_sec*1000000;
    current_timestamp.tv_sec = time_sec;
    current_timestamp.tv_usec = time_usec;
}
