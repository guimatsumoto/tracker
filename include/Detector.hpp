#ifndef DETECTOR_H
#define DETECTOR_H

#include <string>
#include <vector>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <experimental/filesystem>
#include <unistd.h>
#include <cmath>

// Local files
#include "structures.hpp"
#include "stream/RGBDStream.hpp"
#include "stream/DetectionStream.hpp"
#include "viewer/GLViewer.hpp"
//#include "tracking/person_tracker.h"

class Detector {
	public:
		Detector();
		~Detector();

		void setCameraIntrinsics(float fx_, float fy_, float cx_, float cy_);

		void trackOnDetections();

        // emulateTracker is used to test the 3D viewer
        std::vector<tracker::PeoplePose> emulateTracker(std::vector<Eigen::Vector4f> &people);

        // Given a set of 2d points and an image, we estimate depth
		std::vector<Eigen::Vector4f> estimate_keypoints_depth(std::vector<Eigen::Vector3f> pose_keypoints, cv::Mat &depth_frame);


		//void extractDepth();
		//void track(cv::Mat &im);
		void test();

	private:
		// Camera Intrinsics
		float fx, fy, cx, cy;

		// DEPTH EXTRACTION
		int number_of_frames = 0;
		std::vector<Eigen::Vector3f> image_keypoints;
		std::vector<Eigen::Vector4f> depth_keypoints;
		int current_frame = 0;

		// TRACKER
		//PersonTracker* tracker = NULL;
		struct timeval detection_time;
    	double min_confidence_initialization = 4.0; //4.0
    	double min_confidence_detections = -2.5; //-2.5
    	double rate = 15.0;
    	double gate_distance_probability = 0.999;
    	double acceleration_variance = 100; //100
    	double position_variance_weight = 30.0; //30.0
    	bool detector_likelihood = true;
    	bool velocity_in_motion_term = false;
    	double detector_weight = -0.25; //-0.25
    	double motion_weight = 0.25;
    	double sec_before_old = 8.0; //8.0
    	double sec_before_fake = 3.4; //2.4 3.4
    	double sec_remain_new = 1.2; //1.2 0.8
    	int detections_to_validate = 3; //3
    	double voxel_size = 0.06; //0.06

        std::vector<tracker::PeoplePose> pose_vector;

		// Vai ser apenas um vetor de float, cabe ao programa separar pessoas
		// Cada pessoa tem 25 keypoints (x, y, z, c), c = 2D confidence
		Eigen::Vector4f get_depth_using_paf(const int &center_i, const int &center_j, std::vector<std::pair<int, int>> limb_direction);
};

#endif // DETECTOR_H
