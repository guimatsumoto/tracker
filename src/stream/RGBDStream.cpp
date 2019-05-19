#include "stream/RGBDStream.hpp"

namespace fs = std::experimental::filesystem;
using namespace std;

RGBDStream::RGBDStream(std::string path_to_img, std::string path_to_depth){
	// local copy
	img_path = path_to_img;
	depth_path = path_to_depth;

	// store file paths
	img_files = get_file_names(img_path);
	depth_files = get_file_names(depth_path);

	// sort file names
	std::sort(img_files.begin(), img_files.end());
	std::sort(depth_files.begin(), depth_files.end());
}

RGBDStream::~RGBDStream(){

}

int RGBDStream::get_num_frames(){
	return img_files.size();
}

cv::Mat RGBDStream::get_next_rgb_image(){
	try{
		rgb_frame_number++;
		return cv::imread(img_files[rgb_frame_number], CV_LOAD_IMAGE_COLOR);
	}catch(std::exception const& e){
		cout << "Error loading RGB image number " << rgb_frame_number << std::endl;
	}
}

cv::Mat RGBDStream::get_next_depth_image(){
	try{
		depth_frame_number++;
		return cv::imread(depth_files[depth_frame_number], CV_LOAD_IMAGE_COLOR);
	}catch(std::exception const& e){
		cout << "Error loading depth image number " << depth_frame_number << std::endl;
	}
}

void RGBDStream::test(){
	std::cout << img_files.size() << " image files" << std::endl;
	std::cout << depth_files.size() << " depth files" << std::endl;

	printf("%s\n", img_path.c_str());
	printf("%s\n", depth_path.c_str());
}


