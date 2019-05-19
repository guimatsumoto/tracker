#ifndef DETECTOR_H
#define DETECTOR_H

#include "sl_core/ai/ai_release.hpp"

#include "sl_core/ai/LibAILoader.hpp"
#include "sl_core/ai/skeleton/tracking/person_tracker.h"

#ifdef CORE_COMPILATION
#if defined(_WIN32)
#define SL_AI_EXPORT __declspec(dllexport)
#elif __GNUC__
#define SL_AI_EXPORT __attribute__((visibility("default")))
#else
#define SL_AI_EXPORT
#endif
#else
#define SL_AI_EXPORT
#endif

#if defined(_WIN32)
const std::string TMP_MODEL_PATH = std::string(getenv("TEMP"))+"/Gh896AxRMQWC/"; // random name to limit any sneak peak at the files #security
const std::string CMD_FOLDER_CREATION = "mkdir ";
const std::string CMD_RENAME = "move /Y ";
#else
const std::string TMP_MODEL_PATH = "/tmp/.x6Gh896AxRMQWC/"; // random name to limit any sneak peak at the files #security
const std::string CMD_FOLDER_CREATION = "mkdir -p ";
const std::string CMD_RENAME = "mv -f ";
#endif

#define POSE_MAX_PEOPLE 96
#define NET_OUT_CHANNELS 78 // COCO = 57 // BODY_25 = 78

static int counter_global = 0;
static std::map<int, int> serial_to_id;

class SL_AI_EXPORT Detector {
public:

    Detector();
    ~Detector();

    void init(int instance_id, std::string path_cfg, std::string path_model);
    void init_v2(int instance_id, unsigned int net_width, unsigned int net_height, std::string path_compressed_model);

    void close();

    bool detectAndTrack(sl::Mat img, sl::Mat depth, timeval time, sl::Transform pose = sl::Transform());

    // World reference frame, 2D data NOT set, unit and coordinate system define by depth and pose
    std::vector<zed_tracking::PeopleSkeletonOutput> getResults();

    sl::float3 getGazeDirection(int id);

    bool getIsInit() const {
        return isInit;
    }

    void render_pose_keypoints(sl::Mat& frame, std::vector<int> keyshape, const float threshold);

    void setCalibParams(float &fx_, float &fy_, float &cx_, float &cy_, float unit_scale = 1);


    // Static functions
    static unsigned int generateInstanceID(std::string camera_serial, std::string model_name);
    static bool checkFile(std::string path);

    static bool inflateModel(std::string &archive, std::string path_cfg, std::string path_model);
    static bool deflateModel(std::string &archive, std::string path_cfg, std::string path_model);

private:

    int instance_id = 0;
    std::string path_cfg, path_model;

    static void removeFile(std::string path);

    void allocateMemory();
    void releaseMemory();

    bool /*valid*/ getXYZWorld(int i, int j, sl::float4 &value);

    void initNet(std::string path_cfg, std::string path_model);
    void initTracker();
    float *runNet(float *indata);

    bool isInit = false;

    const float MAX_DISTANCE_LIMB = .8; //0.8 1.;
    const float MAX_DISTANCE_CENTER = 1.5; //1.5 1.8;
    const sl::UNIT unit = sl::UNIT_METER;

    // NETWORK
    AILoader aiLoader;
    //network *net = NULL;

    int net_inw = 0;
    int net_inh = 0;
    int net_outw = 0;
    int net_outh = 0;
    float s_w = 0.0f;
    float s_h = 0.0f;
    float *p_netin_data = NULL;
    float *p_heatmap_peaks = NULL;
    float *p_heatmap = NULL;

    // Variables for Pose darknet
    std::vector<float> keypoints;
    std::vector<int> shape;
    float scale = 0.0f;
    cv::Mat netInputArray; //potentially not necessary
    cv::Mat create_netsize_im(const cv::Mat &im, const int netw, const int neth, float *scale, float *s_w, float *s_h);
    void find_heatmap_peaks(const float *src, float *dst, const int SRCW, const int SRCH, const int SRC_CH, const float TH, int s_w, int s_h);
    void connect_bodyparts(std::vector<float>& pose_keypoints, const float* const map, const float* const peaks, int mapw, int maph, const int inter_min_above_th, const float inter_th, const int min_subset_cnt, const float min_subset_score, std::vector<int>& keypoint_shape);
    std::vector<sl::float4> estimate_keypoints_depth(std::vector<float> poseKeypoints);
    sl::float4 PAF_getPatchIdx(const int &center_i, const int &center_j, std::vector<std::pair<int, int>> limb_direction);

    // TRACKER
    std::vector<sl::float4> depth_keypoints;
    zed_tracking::PersonTracker* tracker = NULL;
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

    sl::Mat currentDepth;
    uint64_t current_ts;
    sl::Transform currentPose = sl::Transform::identity();
    float fx_, fy_/*inverse*/, cx, cy;
    float depth_scale = 1;

};

#endif /* DETECTOR_HPP */
