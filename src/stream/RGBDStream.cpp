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

bool RGBDStream::has_more_rgb_images(){
    return (rgb_frame_number < get_num_frames() - 1);
}

bool RGBDStream::has_more_depth_images(){
    return (depth_frame_number < get_num_frames() - 1);
}

cv::Mat RGBDStream::get_next_rgb_image(){
    if (has_more_rgb_images()){
    	try{
	    	rgb_frame_number++;
		    return cv::imread(img_files[rgb_frame_number], CV_LOAD_IMAGE_COLOR);
    	}catch(std::exception const& e){
	    	cout << "Error loading RGB image number " << rgb_frame_number << std::endl;
    	}
    }else{
        std::cout << "There are no more RGB frames" << std::endl;
        return cv::Mat();
    }
}

cv::Mat RGBDStream::get_next_depth_image(){
    if (has_more_depth_images()){
    	try{
	    	depth_frame_number++;
		    //return cv::imread(depth_files[depth_frame_number], CV_LOAD_IMAGE_COLOR);
            return cv::imread(depth_files[depth_frame_number], CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
    	}catch(std::exception const& e){
	    	cout << "Error loading depth image number " << depth_frame_number << std::endl;
    	}
    }else{
        std::cout << "There are no more depth frames" << std::endl;
        return cv::Mat();
    }
}

void RGBDStream::test(){
	std::cout << img_files.size() << " image files" << std::endl;
	std::cout << depth_files.size() << " depth files" << std::endl;

	printf("%s\n", img_path.c_str());
	printf("%s\n", depth_path.c_str());
}


