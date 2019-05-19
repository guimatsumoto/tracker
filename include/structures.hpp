#ifndef STRUCT_H
#define STRUCT_H

struct float3{
	float x;
	float y;
	float c;
};

struct float4{
	float x;
	float y;
	float z;
	float c;
};

// This will be the final output structure, after tracking is done
struct TrackedPoseOutput {
	unsigned int id;

	uint64_t ts;

	float3 barycenter;

	std::vector<int> keypoints_links;
	int keypoint_number;

	std::vector<float3> keypoints_2d;
	int keypoints_2d_number = 0;

	float3 speed;

	std::vector<int> img_position;
	std::vector<int> world_bbox_links;

	float3 gaze_direction;
	float3 body_orientation;
};

#endif // STRUCT_H
