#include <thread>
#include <atomic>

#include "Detector.hpp"
#include "viewer/GLViewer.hpp"
#include "stream/RGBDStream.hpp"
#include "stream/DetectionStream.hpp"
#include "stream/TimeStream.hpp"

// Definition of depth camera intrinsics for depth extraction
#define KTP_CAMERA_INTRINSICS_FX 525.0
#define KTP_CAMERA_INTRINSICS_FY 525.0
#define KTP_CAMERA_INTRINSICS_CX 319.5
#define KTP_CAMERA_INTRINSICS_CY 239.5
#define MENSA_CAMERA_INTRINSICS_FX 594.214342
#define MENSA_CAMERA_INTRINSICS_FY 591.040536
#define MENSA_CAMERA_INTRINSICS_CX 339.307809
#define MENSA_CAMERA_INTRINSICS_CY 242.739137

// Definition of rendering options
#define RENDER_RGB true
#define RENDER_DEPTH true
#define MAX_DISTANCE_CENTER 1.5
#define MAX_DISTANCE_LIMB 1

// Define which dataset to use
#define USE_KTP_DATASET

// Enable tracker
//#define USE_TRACKER

// Define GLViewer -> it has to be global (weird) as in GLUT things
// only make sense if they are global
GLViewer viewer;

// thread callback for different modules
void tracker_run(Detector &d,
                 std::vector<Eigen::Vector3f> &image_kp,
                 std::vector<Eigen::Vector4f> &depth_kp,
                 std::vector<tracker::PeoplePose> &poses,
                 cv::Mat &depth_im);
void stream_run(std::vector<Eigen::Vector3f> &image_kp,
                cv::Mat &rgb_im,
                cv::Mat &depth_im,
                int &current_frame);
void render_run(std::vector<tracker::PeoplePose> &pose,
                PeoplesObject &po,
                int &current_frame,
                cv::Mat &rgb_im,
                cv::Mat &depth_im);
std::atomic_bool quit(0);
std::atomic_bool has_rgbd_detection(0);
std::atomic_bool has_tracked_poses(0);
std::thread stream_callback, tracker_callback, render_callback;

// aux functions
void closeGlut();
void fill_people_object_for_opengl(std::vector<tracker::PeoplePose> &poseKeypoints, PeoplesObject &po);
void fill_test_cube(PeoplesObject &po);

int main(){

	// RGB and depth frames visualization
	cv::Mat rgb_frame, depth_frame;

	// Depth extraction and tracking
	std::vector<Eigen::Vector3f> image_keypoints;
	std::vector<Eigen::Vector4f> depth_keypoints;
	std::vector<tracker::PeoplePose> pose_vector;

    Detector det;

    PeoplesObject peopleObj;

    int current_frame = 0;

    viewer.init();

    stream_callback = std::thread(stream_run,
                                  std::ref(image_keypoints),
                                  std::ref(rgb_frame),
                                  std::ref(depth_frame),
                                  std::ref(current_frame));

    tracker_callback = std::thread(tracker_run,
                                   std::ref(det),
                                   std::ref(image_keypoints),
                                   std::ref(depth_keypoints),
                                   std::ref(pose_vector),
                                   std::ref(depth_frame));

    render_callback = std::thread(render_run,
                                  std::ref(pose_vector),
                                  std::ref(peopleObj),
                                  std::ref(current_frame),
                                  std::ref(rgb_frame),
                                  std::ref(depth_frame));

    glutCloseFunc(closeGlut);
    glutMainLoop();


    std::cout << "Track on detection finished." << std::endl;

	return 1;
}

void tracker_run(Detector &d,
                 std::vector<Eigen::Vector3f> &image_kp,
                 std::vector<Eigen::Vector4f> &depth_kp,
                 std::vector<tracker::PeoplePose> &poses,
                 cv::Mat &depth_im){

    d = Detector();
    TimeStream time_stream(30.f);
#ifdef USE_KTP_DATASET
    d.setCameraIntrinsics(KTP_CAMERA_INTRINSICS_FX,
                KTP_CAMERA_INTRINSICS_FY,
                KTP_CAMERA_INTRINSICS_CX,
                KTP_CAMERA_INTRINSICS_CY);
#else
    d.setCameraIntrinsics(MENSA_CAMERA_INTRINSICS_FX,
                MENSA_CAMERA_INTRINSICS_FY,
                MENSA_CAMERA_INTRINSICS_CX,
                MENSA_CAMERA_INTRINSICS_CY);
#endif

    while (!quit){

        if (!has_rgbd_detection){
            usleep(500);
            continue;
        }

#ifndef USE_TRACKER
        depth_kp = d.estimate_keypoints_depth(image_kp, depth_im);
#if 0
        for (unsigned i = 0; i < depth_kp.size()/25; i++){
            printf("person %d\n", i);
            for (unsigned part = 0; part < 25; part++){
                printf("kp %d: (%f, %f, %f)\n", part,
                                                depth_kp[25*i + part](0),
                                                depth_kp[25*i + part](1),
                                                depth_kp[25*i + part](2));
            }
        }
#endif
    poses = d.emulateTracker(depth_kp);
#else // #ifdef USE_TRACKER
    // Not using cam pose, only gonna do it if there's time
    d.trackOnDetections(image_kp, depth_im, time_stream.get_next());
    depth_kp = d.getDepthKeypoints();
    poses = d.getTrackedPeople();
#if 0
    for (unsigned i = 0; i < depth_kp.size()/25; i++){
        printf("person %d\n", i);
        for (unsigned part = 0; part < 25; part++){
            printf("kp %d: (%f, %f, %f)\n", part,
                                            depth_kp[25*i + part](0),
                                            depth_kp[25*i + part](1),
                                            depth_kp[25*i + part](2));
        }
    }
#endif
#endif

        has_tracked_poses = true;
    }
}

void render_run(std::vector<tracker::PeoplePose> &pose,
                PeoplesObject &po,
                int &current_frame,
                cv::Mat &rgb_im,
                cv::Mat &depth_im){
    while (!quit){

        if (!has_tracked_poses){
            usleep(500);
            continue;
        }

 	    fill_people_object_for_opengl(pose, po);
        //fill_test_cube(po);

	    if (RENDER_RGB && rgb_im.rows>0){
            cv::putText(rgb_im, "Frame: " + std::to_string(current_frame+1), cvPoint(30, 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 200, 250));
            cv::imshow("Pose", rgb_im);
            cv::moveWindow("Pose", 120, 550);
        }

        if (RENDER_DEPTH && depth_im.rows>0){
            cv::imshow("Depth", depth_im);
            cv::moveWindow("Depth", 120, 45);
        }

        if (RENDER_RGB || RENDER_DEPTH){
            cv::waitKey(30);
        }

        viewer.update(po);


        // Simulate 30 FPS
        //usleep(5000);

        has_tracked_poses = false;
        has_rgbd_detection = false;
    }
}

void stream_run(std::vector<Eigen::Vector3f> &image_kp,
                cv::Mat &rgb_im,
                cv::Mat &depth_im,
                int &current_frame){
#ifdef USE_KTP_DATASET
    // Setup images and detections path
    std::string img_path = "/home/gkmatsumoto/tcc/datasets/sequences/ktp_still/img/";
	std::string depth_path = "/home/gkmatsumoto/tcc/datasets/ktp_dataset/images/Still/depth/";
	std::string detection_path = "/home/gkmatsumoto/tcc/datasets/sequences/ktp_still/openpose/output/";
#else
    // Setup images and detections path
	std::string img_path = "/home/gkmatsumoto/tcc/datasets/sequences/mensa_0/img/";
	std::string depth_path = "/home/gkmatsumoto/tcc/datasets/mensa_seq0_1.1/depth_0/";
	std::string detection_path = "/home/gkmatsumoto/tcc/datasets/sequences/mensa_0/content/openpose/output/";
#endif

	// Setup RGBD and Detection streams
	RGBDStream rgbd_stream(img_path, depth_path);
	DetectionStream detection_stream(detection_path);

	int number_of_frames = rgbd_stream.get_num_frames();

    // loop all images and detections
    while (current_frame < number_of_frames){

        while (has_rgbd_detection)
            usleep(500);

        // Get corresponding RGB and Depth frames
        rgb_im = rgbd_stream.get_next_rgb_image();
        depth_im = rgbd_stream.get_next_depth_image();

        // Get detections from this frame
        image_kp = detection_stream.get_next_detection();

        current_frame++;

        // Tell other threads that images and detection are ready;
        has_rgbd_detection = true;

        // Simulate 30 FPS
        usleep(10000);
    }

    quit = true;
}

void closeGlut(){
    quit = true;
    stream_callback.join();
    tracker_callback.join();
    render_callback.join();
    viewer.exit();
}

void fill_people_object_for_opengl(std::vector<tracker::PeoplePose> &poseKeypoints, PeoplesObject &po){
    const auto numberBodyParts = 25;
    const auto numberPeopleDetected = poseKeypoints.size();

    const std::vector<int> partsLink = {
        0, 1, 1, 2, 2, 3, 3, 4, 1, 5, 5, 6, 6, 7, 1, 8, 8, 9, 9, 10,
        10, 11, 11, 22, 11, 24, 8, 12, 12, 13, 13, 14, 14, 19, 14, 21,
    };

    Eigen::Vector4f v1, v2;
    std::vector<tracker::float3> vertices, clr;

    for (unsigned person = 0; person < numberPeopleDetected; person++){
        Eigen::Vector3f center_gravity = poseKeypoints[person].barycenter;
        std::vector<Eigen::Vector4f> kps = poseKeypoints[person].keypoints_3d;
        bool visualize = true;
        for (unsigned part = 0; part < partsLink.size(); part += 2){
            v1 = Eigen::Vector4f(kps[partsLink[part]]);
            v2 = Eigen::Vector4f(kps[partsLink[part+1]]);
            visualize = true;

            float distance = sqrt((v1(0) - v2(0)) * (v1(0) - v2(0)) +
                                  (v1(1) - v2(1)) * (v1(1) - v2(1)) +
                                  (v1(2) - v2(2)) * (v1(2) - v2(2)));

            float distance_gravity_center = sqrt( pow((v2(0) + v1(0))*0.5f - center_gravity(0), 2) +
                                                  pow((v2(1) + v1(1))*0.5f - center_gravity(1), 2) +
                                                  pow((v2(2) + v1(2))*0.5f - center_gravity(2), 2));
            //if (isfinite(distance_gravity_center) && distance < MAX_DISTANCE_LIMB){
            if (isfinite(distance_gravity_center) &&
                isfinite(distance) &&
                (v1(3) > .5) && (v2(3) > .5)){
                vertices.push_back(tracker::float3{v1(0), v1(1), v1(2)});
#if 0
                std::cout << vertices.back().x << " "
                          << vertices.back().y << " "
                          << vertices.back().c << std::endl;
#endif
                vertices.push_back(tracker::float3{v2(0), v2(1), v2(2)});
#if 0
                std::cout << vertices.back().x << " "
                          << vertices.back().y << " "
                          << vertices.back().c << std::endl;
#endif
                clr.push_back(generateColor(poseKeypoints[person].id));
                clr.push_back(generateColor(poseKeypoints[person].id));
            }
        }
    }
    po.setVert(vertices, clr);
}

void fill_test_cube(PeoplesObject &po){
    std::vector<tracker::float3> vertices, clr;

    float side = 1.5;

    // vert 1
    vertices.emplace_back(tracker::float3{0, .5, 0});
    vertices.emplace_back(tracker::float3{side, .5, 0});
    clr.push_back(generateColor(-1));
    clr.push_back(generateColor(-1));
    // vert 2
    vertices.emplace_back(tracker::float3{side, .5, 0});
    vertices.emplace_back(tracker::float3{side, .5, -side});
    clr.push_back(generateColor(-1));
    clr.push_back(generateColor(-1));
    // vert 3
    vertices.emplace_back(tracker::float3{side, .5, -side});
    vertices.emplace_back(tracker::float3{0, .5, -side});
    clr.push_back(generateColor(-1));
    clr.push_back(generateColor(-1));
    // vert 4
    vertices.emplace_back(tracker::float3{0, .5, -side});
    vertices.emplace_back(tracker::float3{0, .5, 0});
    clr.push_back(generateColor(-1));
    clr.push_back(generateColor(-1));
    // vert 5
    vertices.emplace_back(tracker::float3{0, .5, 0});
    vertices.emplace_back(tracker::float3{0, .5+side, 0});
    clr.push_back(generateColor(-1));
    clr.push_back(generateColor(-1));
    // vert 6
    vertices.emplace_back(tracker::float3{side, .5, 0});
    vertices.emplace_back(tracker::float3{side, .5+side, 0});
    clr.push_back(generateColor(-1));
    clr.push_back(generateColor(-1));
    // vert 7
    vertices.emplace_back(tracker::float3{side, .5, -side});
    vertices.emplace_back(tracker::float3{side, .5+side, -side});
    clr.push_back(generateColor(-1));
    clr.push_back(generateColor(-1));
    // vert 8
    vertices.emplace_back(tracker::float3{0, .5, -side});
    vertices.emplace_back(tracker::float3{0, .5+side, -side});
    clr.push_back(generateColor(-1));
    clr.push_back(generateColor(-1));
    // vert 9
    vertices.emplace_back(tracker::float3{0, .5+side, 0});
    vertices.emplace_back(tracker::float3{side, .5+side, 0});
    clr.push_back(generateColor(-1));
    clr.push_back(generateColor(-1));
    // vert 10
    vertices.emplace_back(tracker::float3{side, .5+side, 0});
    vertices.emplace_back(tracker::float3{side, .5+side, -side});
    clr.push_back(generateColor(-1));
    clr.push_back(generateColor(-1));
    // vert 11
    vertices.emplace_back(tracker::float3{side, .5+side, -side});
    vertices.emplace_back(tracker::float3{0, .5+side, -side});
    clr.push_back(generateColor(-1));
    clr.push_back(generateColor(-1));
    // vert 12
    vertices.emplace_back(tracker::float3{0, .5+side, -side});
    vertices.emplace_back(tracker::float3{0, .5+side, 0});
    clr.push_back(generateColor(-1));
    clr.push_back(generateColor(-1));
    po.setVert(vertices, clr);

    //printf(" %f %f %f\n", vertices[23].x, vertices[23].y, vertices[23].c);

    //printf("vertex size: %d\n", vertices.size());
}
