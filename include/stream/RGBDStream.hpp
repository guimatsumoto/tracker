#ifndef RGBD_STREAM_H
#define RGBD_STREAM_H

#include <string>
#include <vector>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <experimental/filesystem>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <bits/stdc++.h>
#include "utils.hpp"

class RGBDStream {
	public:
		RGBDStream(std::string path_to_img, std::string path_to_depth);
		~RGBDStream();

		int get_num_frames();

		cv::Mat get_next_rgb_image();
		cv::Mat get_next_depth_image();

		void test();
	private:
		int rgb_frame_number = 0;
		int depth_frame_number = 0;

		// RGBD image paths
		std::string img_path;
		std::string depth_path;

		std::vector<std::string> img_files;
		std::vector<std::string> depth_files;

};

#endif // RGBD_STREAM_H
