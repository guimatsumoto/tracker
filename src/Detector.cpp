#include "Detector.hpp"

namespace fs = std::experimental::filesystem;

Detector::Detector(std::string path_to_img, std::string path_to_depth, std::string path_to_detections)
	: rgbd_stream(path_to_img, path_to_depth)
	, detection_stream(path_to_detections)
	, fx(0), fy(0), cx(0), cy(0) {

	number_of_frames = rgbd_stream.get_num_frames();
	setRenderOptions(1, 1, 1);

	// Test stream initialization
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

void Detector::setRenderOptions(bool render_rgb, bool render_depth, bool render_3d){
	render_rgb_image = render_rgb;
	render_depth_image = render_depth;
	render_3d_poses = render_3d;
}

void Detector::trackOnDetections(){
	// Activate 3d pose viewer if flag is true
	if (render_3d_poses){
		viewer.init();
        glutCloseFunc(closeGlut);
        glutMainLoop();
    }
	int current_frame = 0;
	while (current_frame <= number_of_frames){

		std::cout << "Frame: " << current_frame << std::endl;

		// get rgb and depth frames
		rgb_frame = rgbd_stream.get_next_rgb_image();
		depth_frame = rgbd_stream.get_next_depth_image();
		image_keypoints = detection_stream.get_next_detection();

		//printf("avant depth dims: (%d, %d)\n", depth_frame.cols, depth_frame.rows);

		// extract depth
		depth_keypoints = estimate_keypoints_depth(image_keypoints);

        // track
        pose_vector = emulateTracker(depth_keypoints);

        // prepare opengl structure to show 3d poses
        fill_people_object_for_opengl(pose_vector);

		// test depth
#if 0
		if (depth_keypoints.size() > 1){
			for (int i = 0; i < depth_keypoints.size()/25; i++){
				printf("person %d\n", i);
				for (int j = 0; j < 25; j++){
					std::cout << "kp " << j << ": ("
						  << depth_keypoints[25*i+j].x
						  << ", "
						  << depth_keypoints[25*i+j].y
						  << ", "
						  << depth_keypoints[25*i+j].z
						  << ") - 2d confidence = "
						  << depth_keypoints[25*i+j].c
						  << std::endl;
				}
			}
		}
#endif

		// rendering routines
		if (render_rgb_image){
			cv::putText(rgb_frame, "Frame: " + std::to_string(current_frame + 1), cvPoint(30, 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250));
			// display rgb and depth frames
			cv::imshow("Pose", rgb_frame);
			cv::moveWindow("Pose", 120, 550);
		}
		if (render_depth_image){
			cv::imshow("Depth", depth_frame);
			cv::moveWindow("Depth", 120, 45);
		}
		if (render_rgb_image || render_depth_image){
			cv::waitKey(10);
		}
		if (render_3d_poses){
			viewer.update(peopleObj);
		}

		current_frame++;
		// simulate 30 FPS
		usleep(10000);
	}

	std::cout << "Track on detection finished." << std::endl;

	// Close everything, clear space
}

std::vector<Eigen::Vector4f> Detector::estimate_keypoints_depth(std::vector<Eigen::Vector3f> pose_keypoints){
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
			depth_keypoints.push_back(kp);
		}
	}
	return depth_keypoints;
}

Eigen::Vector4f get_depth_using_paf(const int &center_i, const int &center_j, std::vector<std::pair<int, int>> limb_direction){

}

void Detector::fill_people_object_for_opengl(std::vector<tracker::PeoplePose> &poseKeypoints){
    const auto numberBodyParts = 25;
    const auto numberPeopleDetected = poseKeypoints.size();

    const std::vector<int> partsLink = {
        0, 1, 1, 2, 2, 3, 3, 4, 1, 5, 5, 6, 6, 7, 1, 8, 8, 9, 9, 10,
        10, 11, 11, 22, 11, 24, 8, 12, 12, 13, 13, 14, 14, 19, 14, 21,
    };

    Eigen::Vector4f v1, v2;
    std::vector<Eigen::Vector3f> vertices, clr;

    for (unsigned person = 0; person < numberPeopleDetected; person++){
        Eigen::Vector3f center_gravity = poseKeypoints[person].barycenter;
        std::vector<Eigen::Vector4f> kps = poseKeypoints[person].keypoints_3d;
        bool visualize = true;
        for (unsigned part = 0; part < partsLink.size(); part += 2){
            v1 = Eigen::Vector4f(kps[partsLink[part]]);
            v2 = Eigen::Vector4f(kps[partsLink[part+1]]);
            visualize = true;
        }

        float distance = sqrt((v1(0) - v2(0)) * (v1(0) - v2(0)) +
                              (v1(1) - v2(1)) * (v1(1) - v2(1)) +
                              (v1(2) - v2(2)) * (v1(2) - v2(2)));

        float distance_gravity_center = sqrt( pow((v2(0) + v1(0))*0.5f - center_gravity(0), 2) +
                                              pow((v2(1) + v1(1))*0.5f - center_gravity(1), 2) +
                                              pow((v2(2) + v1(2))*0.5f - center_gravity(2), 2));

        if (isfinite(distance_gravity_center) && distance < MAX_DISTANCE_LIMB){
            vertices.emplace_back(Eigen::Vector3f(v1(0), v1(1), v1(2)));
            vertices.emplace_back(Eigen::Vector3f(v2(0), v2(1), v2(2)));
            clr.push_back(generateColor(poseKeypoints[person].id));
            clr.push_back(generateColor(poseKeypoints[person].id));
        }
    }
    peopleObj.setVert(vertices, clr);
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

void Detector::closeGlut(){
    viewer.exit();
}
