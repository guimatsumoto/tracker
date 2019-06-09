#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <experimental/filesystem>
#include <opencv2/opencv.hpp>

#ifndef M_PI
#define M_PI 3.1416f
#endif

std::vector<std::string> get_file_names(std::string dir);

inline double diff_s(timeval t1, timeval t2){
    return (((t1.tv_sec - t2.tv_sec)*(double)1000000) +
             (t1.tv_usec - t2.tv_usec)) / 1000000;
}

inline double diff_ms(timeval t1, timeval t2){
    return (((t1.tv_sec - t2.tv_sec)*(double)1000000)+
             (t1.tv_usec - t2.tv_usec)) / 1000;
}

inline double timeval_to_sec(timeval t){
    return (double) t.tv_sec + (double) (t.tv_usec / 1000000);
}

#endif // UTILS_H
