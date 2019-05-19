#include "Detector.hpp"

int main(){
	std::string img_path = "/home/gkmatsumoto/tcc/datasets/sequences/ktp_still/img/";
	std::string depth_path = "/home/gkmatsumoto/tcc/datasets/ktp_dataset/images/Still/depth/";
	std::string detection_path = "/home/gkmatsumoto/tcc/datasets/sequences/ktp_still/openpose/output/";
	Detector det(img_path, depth_path, detection_path);

	det.trackOnDetections();

	return 1;
}
