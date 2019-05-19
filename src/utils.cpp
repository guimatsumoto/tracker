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
