#include "stream/DetectionStream.hpp"

namespace fs = std::experimental::filesystem;
using namespace std;

DetectionStream::DetectionStream(std::string path_to_detection){
	detection_path = path_to_detection;

	detection_files = get_file_names(detection_path);
	std::sort(detection_files.begin(), detection_files.end());
}

DetectionStream::~DetectionStream(){

}

int DetectionStream::get_num_frames(){
	return detection_files.size();
}

std::vector<Eigen::Vector3f> DetectionStream::get_next_detection(){
	FILE *fp;
	long lSize;
	char *buffer;
	std::vector<Eigen::Vector3f> detected_poses;
	try{
		// Reading the whole json into memory
		fp = fopen(detection_files[frame_number].c_str(), "rb");

		fseek(fp, 0L, SEEK_END);
		lSize = ftell(fp);
		rewind(fp);

		buffer = (char*) calloc(1, lSize+1);

		fread(buffer, lSize, 1, fp);

		// Handling with rapidjson
		rapidjson::Document d;
		d.Parse(buffer);
		for (unsigned i = 0; i < d["people"].Size(); i++){
			for (unsigned j = 0; j < tracker::PoseJoints::SIZE; j++){
				Eigen::Vector3f keypoint(static_cast<float>(d["people"][i]["pose_keypoints_2d"][3*j].GetDouble()),
                                         static_cast<float>(d["people"][i]["pose_keypoints_2d"][3*j+1].GetDouble()),
                                         static_cast<float>(d["people"][i]["pose_keypoints_2d"][3*j+2].GetDouble()));

				detected_poses.push_back(keypoint);
			}
		}

		fclose(fp);
		free(buffer);
		frame_number++;
		return detected_poses;
	}catch(std::exception const& e){
		std::cout << "Error loading detection " << frame_number << std::endl;
	}
}

void DetectionStream::test(){
	std::cout << detection_files.size() << " detection files" << std::endl;

	printf("%s\n", detection_path.c_str());
}
