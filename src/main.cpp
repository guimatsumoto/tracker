#include "Detector.hpp"
#include "viewer/GLViewer.hpp"
#include "stream/RGBDStream.hpp"
#include "stream/DetectionStream.hpp"

// Definition of depth camera intrinsics for depth extraction
#define KTP_CAMERA_INTRINSICS_FX 525.0
#define KTP_CAMERA_INTRINSICS_FY 525.0
#define KTP_CAMERA_INTRINSICS_CX 319.5
#define KTP_CAMERA_INTRINSICS_CY 239.5
#define MENSA_CAMERA_INTRINSICS_FX 594.214342
#define MENSA_CAMERA_INTRINSICS_FY 591.040536
#define MENSA_CAMERA_INTRINSICS_CX 339.307809
#define MENSA_CAMERA_INTRINSICS_CY 242.739137

// Define which dataset to use
#define USE_KTP_DATASET

int main(){

#ifdef USE_KTP_DATASET
	std::string img_path = "/home/gkmatsumoto/tcc/datasets/sequences/ktp_still/img/";
	std::string depth_path = "/home/gkmatsumoto/tcc/datasets/ktp_dataset/images/Still/depth/";
	std::string detection_path = "/home/gkmatsumoto/tcc/datasets/sequences/ktp_still/openpose/output/";
	Detector det(img_path, depth_path, detection_path);
	det.setCameraIntrinsics(KTP_CAMERA_INTRINSICS_FX,
				KTP_CAMERA_INTRINSICS_FY,
				KTP_CAMERA_INTRINSICS_CX,
				KTP_CAMERA_INTRINSICS_CY);
#else
	std::string img_path = "/home/gkmatsumoto/tcc/datasets/sequences/mensa_0/img/";
	std::string depth_path = "/home/gkmatsumoto/tcc/datasets/mensa_seq0_1.1/depth_0/";
	std::string detection_path = "/home/gkmatsumoto/tcc/datasets/sequences/mensa_0/content/openpose/output/";
	Detector det(img_path, depth_path, detection_path);
	det.setCameraIntrinsics(MENSA_CAMERA_INTRINSICS_FX,
				MENSA_CAMERA_INTRINSICS_FY,
				MENSA_CAMERA_INTRINSICS_CX,
				MENSA_CAMERA_INTRINSICS_CY);
#endif

//    GLViewer viewer;
//    viewer.init();

    

	det.trackOnDetections();

	return 1;
}
