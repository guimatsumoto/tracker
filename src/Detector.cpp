#include "Detector.hpp"
//#include <opencv2/opencv.hpp>

namespace fs = std::experimental::filesystem;

Detector::Detector(std::string path_to_img, std::string path_to_depth, std::string path_to_detections)
	: rgbd_stream(path_to_img, path_to_depth)
	, detection_stream(path_to_detections) {

	number_of_frames = rgbd_stream.get_num_frames();

	// Test stream initialization
	//rgbd_stream.test();
	//detection_stream.test();
}

Detector::~Detector(){

}

void Detector::test(){
	//printf("%s\n", detection_path.c_str());
}

void Detector::trackOnDetections(){
	int current_frame = 0;
	cv::Mat rgb_frame, depth_frame;
	while (current_frame <= number_of_frames){

		std::cout << "Frame: " << current_frame << std::endl;

		// get rgb and depth frames
		rgb_frame = rgbd_stream.get_next_rgb_image();
		depth_frame = rgbd_stream.get_next_depth_image();
		image_keypoints = detection_stream.get_next_detection();

		for (unsigned i = 0; i < image_keypoints.size(); i++){
			printf("%f,%f,%f\n", image_keypoints[i].x, image_keypoints[i].y, image_keypoints[i].c);
		}

		cv::putText(rgb_frame, "Frame: " + std::to_string(current_frame + 1), cvPoint(30, 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250));

		// display rgb and depth frames
		cv::imshow("Pose", rgb_frame);
		cv::moveWindow("Pose", 120, 550);

		cv::imshow("Depth", depth_frame);
		cv::moveWindow("Depth", 120, 45);

		cv::waitKey(10);

		current_frame++;
		// simulate 30 FPS
		usleep(10000);
	}

	std::cout << "Track on detection finished." << std::endl;
}

std::vector<float4> estimate_keypoints_depth(std::vector<float3> pose_keypoints){

}

float4 get_depth_using_paf(const int &center_i, const int &center_j, std::vector<std::pair<int, int>> limb_direction){

}
