#ifndef STRUCT_H
#define STRUCT_H

#include <Eigen/Dense>

#include <random>
/*
namespace {
std::random_device rd;
std::mt19937 e2(rd());
std::uniform_real_distribution<> dist(0.1, 1);
std::vector<tracker::float3> color_vect;
}

inline tracker::float3 generateColor(int idx = -1){
    if (idx < 0)
        return tracker::float3{dist(e2), dist(e2), dist(e2)};
    else{
        while (color_vect.size() <= idx) color_vect.emplace_back(tracker::float3{dist(e2), dist(e2), dist(e2)});
        return color_vect.at(idx);
    }
}
*/

namespace tracker{

	typedef struct float2{
		float x;
		float y;
	} float2;

	typedef struct float3{
		float x;
		float y;
		float c;
	} float3;

	typedef struct float4{
		float x;
		float y;
		float z;
		float c;
	} float4;

	typedef struct uint2{
		unsigned i;
		unsigned j;
	} uint2;

	typedef struct uint3{
		unsigned i;
		unsigned j;
		unsigned k;
	} uint3;

	// This will be the final output structure, after tracking is done
	typedef struct PeoplePose {
		unsigned int id;

		uint64_t ts;

		Eigen::Vector3f barycenter;

		std::vector<int> keypoints_links;
		int keypoint_number;

		std::vector<Eigen::Vector3f> keypoints_2d;
		int keypoints_2d_number = 0;

        std::vector<Eigen::Vector4f> keypoints_3d;

		Eigen::Vector3f speed;

		std::vector<int> img_position;
		std::vector<int> world_bbox_links;

		Eigen::Vector3f gaze_direction;
		Eigen::Vector3f body_orientation;
	} TrackedPoseOutput;;

}

namespace {
std::random_device rd;
std::mt19937 e2(rd());
std::uniform_real_distribution<> dist(0.1, 1);
std::vector<tracker::float3> color_vect;
}

inline tracker::float3 generateColor(int idx = -1){
    if (idx < 0)
        return tracker::float3{1., 0, 0};
    else{
        while (color_vect.size() <= idx) color_vect.emplace_back(tracker::float3{(float)dist(e2), (float)dist(e2), (float)dist(e2)});
        return color_vect.at(idx);
    }
}

#endif // STRUCT_H
