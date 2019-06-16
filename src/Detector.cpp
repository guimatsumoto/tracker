#include "Detector.hpp"

namespace fs = std::experimental::filesystem;

void fill_chi_map(std::map<double, double> &chi_map,
                bool velocity_in_motion_term){
    if(velocity_in_motion_term) { // chi square values with state dimension = 4
        chi_map[0.5] = 3.357;
        chi_map[0.75] = 5.385;
        chi_map[0.8] = 5.989;
        chi_map[0.9] = 7.779;
        chi_map[0.95] = 9.488;
        chi_map[0.98] = 11.668;
        chi_map[0.99] = 13.277;
        chi_map[0.995] = 14.860;
        chi_map[0.998] = 16.924;
        chi_map[0.999] = 18.467;
    } else {// chi square values with state dimension = 2
        chi_map[0.5] = 1.386;
        chi_map[0.75] = 2.773;
        chi_map[0.8] = 3.219;
        chi_map[0.9] = 4.605;
        chi_map[0.95] = 5.991;
        chi_map[0.98] = 7.824;
        chi_map[0.99] = 9.210;
        chi_map[0.995] = 10.597;
        chi_map[0.998] = 12.429;
        chi_map[0.999] = 13.816;
    }

}

void fill_chi_map_3d(std::map<double, double>& chi_map, bool velocity_in_motion_term) {
    if(velocity_in_motion_term) {// chi square values with state dimension = 6
        chi_map[0.5] = 5.348157744978145;
        chi_map[0.75] = 7.840834359138661;
        chi_map[0.8] = 8.558055165624488;
        chi_map[0.9] = 10.64489261044355;
        chi_map[0.95] = 12.591393389500972;
        chi_map[0.98] = 15.032781872114004;
        chi_map[0.99] = 16.811720740614913;
        chi_map[0.995] = 18.547667872189702;
        chi_map[0.998] = 20.791761918616732;
        chi_map[0.999] = 22.459125687950245;
    } else { // chi square values with state dimension = 3
        chi_map[0.5] = 2.3659773627544762;
        chi_map[0.75] = 4.108376448120975;
        chi_map[0.8] = 4.641569733033475;
        chi_map[0.9] = 6.251462635371571;
        chi_map[0.95] = 7.814674086652431;
        chi_map[0.98] = 9.837546768092276;
        chi_map[0.99] = 11.345192964279828;
        chi_map[0.995] = 12.838203077572782;
        chi_map[0.998] = 14.796089301316622;
        chi_map[0.999] = 16.26733195007081;
    }
}

Detector::Detector()
	: fx(0), fy(0), cx(0), cy(0) {
    init_tracker();
}

Detector::~Detector(){

}

void Detector::init_tracker(){
    std::map<double, double> chi_map;
    fill_chi_map(chi_map, velocity_in_motion_term);
    double period = 1.0/rate;
    double gate_distance = chi_map.find(gate_distance_probability) != chi_map.end() ? chi_map[gate_distance_probability] : chi_map[0.999];
    double position_variance = position_variance_weight * std::pow(2 * voxel_size, 2) / 12;
    std::vector<double> likelihood_weights;
    likelihood_weights.push_back(detector_weight * chi_map[0.999] / 18.467);
    likelihood_weights.push_back(motion_weight);

    tracker = new tracker::PersonTracker(gate_distance,
                                         detector_likelihood,
                                         likelihood_weights,
                                         velocity_in_motion_term,
                                         min_confidence_initialization,
                                         min_confidence_detections,
                                         sec_before_old,
                                         sec_before_fake,
                                         sec_remain_new,
                                         detections_to_validate,
                                         period,
                                         position_variance,
                                         acceleration_variance,
                                         "world",
                                         false,
                                         false);
    tracker_init = true;
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

bool Detector::trackOnDetections(std::vector<Eigen::Vector3f> pose_keypoints,
                                 cv::Mat &depth_frame,
                                 timeval time,
                                 Eigen::Affine3f cam_pose){
    if (!tracker_init)
        return false;

    current_depth = depth_frame;
    current_pose = cam_pose;
    current_ts = time.tv_sec * 1000000000 + time.tv_usec + 1000;

    // Extracting depth
    depth_keypoints = estimate_keypoints_depth(pose_keypoints, current_depth);

    // Tracking
    // On the french thesis shape came from the cnn directly, here we
    // emulate it
    std::vector<int> shape = {(int)depth_keypoints.size()/25, 25, 0};
    //std::cout << shape[0] << ", " << shape[1] << ", " << shape[2] << std::endl;
    tracker->newFrame(depth_keypoints, shape, time);
    tracker->updateTracks();

    return true;
}

std::vector<tracker::PeoplePose> Detector::getTrackedPeople(){
    auto out = tracker->getTrackedPeople();
    for(auto &it : out) it.ts = current_ts;
    return out;
}

std::vector<Eigen::Vector4f> Detector::getDepthKeypoints(){
    return depth_keypoints;
}

std::vector<Eigen::Vector4f> Detector::estimate_keypoints_depth(std::vector<Eigen::Vector3f> pose_keypoints, cv::Mat &depth_frame){
	std::vector<Eigen::Vector4f> depth_keypoints;
	float x, y, z;
	if (pose_keypoints.size() > 1){
		for (int i = 0; i < pose_keypoints.size(); i++){
			Eigen::Vector4f kp;
            if ((pose_keypoints[i](0) == 0) && (pose_keypoints[i](1) == 0)){
                kp(0) = NAN;
                kp(1) = NAN;
                kp(2) = NAN;
                kp(3) = 0.f;
            }else{
			    // Trivial method
			    z = (float)depth_frame.at<short>((int)pose_keypoints[i](1), (int)pose_keypoints[i](0))/1000;
			    // Stephane Magnenat's method
			    //z = 0.1236*tan(depth_frame.at<short>((int)pose_keypoints[i](1), (int)pose_keypoints[i](0)) / 2842.5 + 1.1863);
			    x = (pose_keypoints[i](0) - cx) * z / fx;
			    y = (pose_keypoints[i](1) - cy) * z / fy;
                if (x + y + z == 0){
                    kp(0) = NAN;
			        kp(1) = NAN;
			        kp(2) = NAN;
			        kp(3) = pose_keypoints[i](2);
                }else{
			        kp(0) = x;
			        kp(1) = y;
			        kp(2) = z;
			        kp(3) = pose_keypoints[i](2);
                }
            }
#if 0
            std::cout << pose_keypoints[i](0) << " " << pose_keypoints[i](1) << " - ";
            std::cout << kp(0) << " " << kp(1) << " " << kp(2) << std::endl;
#endif
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
            aux.keypoints_3d.push_back(Eigen::Vector4f(people[25*i+j]));
            if (people[25*i+j](3) > 0){
                Eigen::Vector3f kp_pos(people[25*i+j](0),
                                       people[25*i+j](1),
                                       people[25*i+j](2));
                if (isfinite(kp_pos(0))){
                    barycenter += kp_pos;
                    valid_kps += 1;
                }
            }
        }
        if (valid_kps > 0)
            barycenter = barycenter / valid_kps;
        else
            barycenter = Eigen::Vector3f(NAN, NAN, NAN);
        aux.barycenter = barycenter;
        ogl_people.push_back(aux);
    }
    return ogl_people;
}
