#ifndef DETECTION_STREAM_H
#define DETECTION_STREAM_H

#include <string>
#include <vector>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <bits/stdc++.h>
#include <cstdio>

// Local includes
#include "utils.hpp"
#include "structures.hpp"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/rapidjson.h"
#include "tracking/pose.h"

class DetectionStream {
	public:
		DetectionStream(std::string path_to_detection);
		~DetectionStream();

		int get_num_frames();

		std::vector<Eigen::Vector3f> get_next_detection();

		void test();
	private:
		int frame_number = 0;

		std::string detection_path;

		std::vector<std::string> detection_files;
};

#endif //DETECTION_STREAM_H
