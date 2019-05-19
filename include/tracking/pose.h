#ifndef POSE_H
#define POSE_H

#include <vector>
#include <array>

namespace tracker {
	enum PoseJoints{
		NOSE = 0,
		NECK,
		RSHOULDER,
		RELBOW,
		RWRIST,
		LSHOULDER,
		LELBOW,
		LWRIST,
		MIDHIP,
		RHIP,
		RKNEE,
		RANKLE,
		LHIP,
		LKNEE,
		LANKLE,
		REYE,
		LEYE,
		REAR,
		LEAR,
		LBIGTOE,
		LSMALLTOE,
		LHEEL,
		RBIGTOE,
		RSMALLTOE,
		RHEEL,
		SIZE
	};

}

#endif // POSE_H
