#include "utils.hpp"

namespace fs = std::experimental::filesystem;
using namespace std;

std::vector<std::string> get_file_names(std::string dir){
        int count = 0;
        std::vector<std::string> file_list;
        for (auto & entry : fs::directory_iterator(dir)){
                if (fs::exists(entry.path()))
                        file_list.push_back(entry.path());
        }
        return file_list;
}

cv::Mat swap_img_bytes(cv::Mat &img){
    int len_byte = img.elemSize();
    typedef char tmp_data_t[len_byte];
    {
        cv::Mat to_swap(img.rows, img.cols, CV_8UC(len_byte), img.data);
        int dtype = img.type();
        cv::Mat merged;
        std::vector<cv::Mat> channels(len_byte);
        cv::split(to_swap, channels);
        std::reverse(channels.begin(), channels.end());
        cv::merge(&channels[0], len_byte, merged);
        merged.addref();
        return cv::Mat(to_swap.rows, to_swap.cols, dtype, merged.data);
    }
}
