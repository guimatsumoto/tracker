#include "Detector.hpp"

namespace fs = std::experimental::filesystem;

Detector::Detector()
	: fx(0), fy(0), cx(0), cy(0) {
    // Stream test
	//rgbd_stream.test();
	//detection_stream.test();
}

Detector::~Detector(){

}

void Detector::test(){
	//printf("%s\n", detection_path.c_str());
}

void Detector::setCameraIntrinsics(float fx_, float fy_, float cx_, float cy_){
	fx = fx_;
	fy = fy_;
	cx = cx_;
	cy = cy_;
}

void Detector::trackOnDetections(){

}

std::vector<Eigen::Vector4f> Detector::estimate_keypoints_depth(std::vector<Eigen::Vector3f> pose_keypoints, cv::Mat &depth_frame){
	std::vector<Eigen::Vector4f> depth_keypoints;
	float x, y, z;
	if (pose_keypoints.size() > 1){
		for (int i = 0; i < pose_keypoints.size(); i++){
			// Trivial method
			z = (float)depth_frame.at<short>((int)pose_keypoints[i](1), (int)pose_keypoints[i](0))/1000;
			// Stephane Magnenat's method
			//z = 0.1236*tan(depth_frame.at<short>((int)pose_keypoints[i].y, (int)pose_keypoints[i].x) / 2842.5 + 1.1863);
			x = (pose_keypoints[i](0) - cx) * z / fx;
			y = (pose_keypoints[i](1) - cy) * z / fy;
			Eigen::Vector4f kp;
			kp(0) = x;
			kp(1) = y;
			kp(2) = z;
			kp(3) = pose_keypoints[i](2);
            std::cout << kp(0) << " " << kp(1) << " " << kp(2) << std::endl;
			depth_keypoints.push_back(kp);
		}
	}
	return depth_keypoints;
}

Eigen::Vector4f get_depth_using_paf(const int &center_i, const int &center_j, std::vector<std::pair<int, int>> limb_direction){

}

std::vector<tracker::PeoplePose> Detector::emulateTracker(std::vector<Eigen::Vector4f> &people){
    std::vector<tracker::PeoplePose> ogl_people;
    for (unsigned i = 0; i < people.size()/25; i++){
        tracker::PeoplePose aux;
        aux.id = -1;
        Eigen::Vector3f barycenter(0, 0, 0);
        unsigned valid_kps;
        for (unsigned j = 0; j < 25; j++){
            aux.keypoints_3d.emplace_back(Eigen::Vector4f(people[25*i+j]));
            if (people[25*i+j](3) > 0){
                Eigen::Vector3f kp_pos(people[25*i+j](0),
                                       people[25*i+j](1),
                                       people[25*i+j](2));
                barycenter += kp_pos;
                valid_kps += 1;
            }
        }
        barycenter = barycenter / valid_kps;
        aux.barycenter = barycenter;
        ogl_people.emplace_back(aux);
    }
    return ogl_people;
}
