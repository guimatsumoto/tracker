#include "sl_core/ai/Detector.hpp"
#include <sys/stat.h>
#include <functional> //for std::hash
#include <iostream>
#include <fstream>

#if defined(LINKS_WITH_EXTERN_OPENCV)
#include <opencv2/opencv.hpp>
#else
#include "sl_core/opencv/cv_wrapper.hpp"
using namespace slutils;
#endif

#include "sl_core/ai/miniz.h"
using namespace std;

cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch(input.getDataType()) {
    case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1;
        break;
    case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2;
        break;
    case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3;
        break;
    case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4;
        break;
    case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1;
        break;
    case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2;
        break;
    case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3;
        break;
    case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4;
        break;
    default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

void fillChiMap(std::map<double, double>& chi_map, bool velocity_in_motion_term) {
    if(velocity_in_motion_term) { // chi square values with state dimension = 4
        chi_map[0.5] = 3.357;
        chi_map[0.75] = 5.385;
        chi_map[0.8] = 5.989;
        chi_map[0.9] = 7.779;
        chi_map[0.95] = 9.488;
        chi_map[0.98] = 11.668;
        chi_map[0.99] = 13.277;
        chi_map[0.995] = 14.860;
        chi_map[0.998] = 16.924;
        chi_map[0.999] = 18.467;
    } else {// chi square values with state dimension = 2
        chi_map[0.5] = 1.386;
        chi_map[0.75] = 2.773;
        chi_map[0.8] = 3.219;
        chi_map[0.9] = 4.605;
        chi_map[0.95] = 5.991;
        chi_map[0.98] = 7.824;
        chi_map[0.99] = 9.210;
        chi_map[0.995] = 10.597;
        chi_map[0.998] = 12.429;
        chi_map[0.999] = 13.816;
    }
}

void fillChiMap3D(std::map<double, double>& chi_map, bool velocity_in_motion_term) {
    if(velocity_in_motion_term) {// chi square values with state dimension = 6
        chi_map[0.5] = 5.348157744978145;
        chi_map[0.75] = 7.840834359138661;
        chi_map[0.8] = 8.558055165624488;
        chi_map[0.9] = 10.64489261044355;
        chi_map[0.95] = 12.591393389500972;
        chi_map[0.98] = 15.032781872114004;
        chi_map[0.99] = 16.811720740614913;
        chi_map[0.995] = 18.547667872189702;
        chi_map[0.998] = 20.791761918616732;
        chi_map[0.999] = 22.459125687950245;
    } else { // chi square values with state dimension = 3
        chi_map[0.5] = 2.3659773627544762;
        chi_map[0.75] = 4.108376448120975;
        chi_map[0.8] = 4.641569733033475;
        chi_map[0.9] = 6.251462635371571;
        chi_map[0.95] = 7.814674086652431;
        chi_map[0.98] = 9.837546768092276;
        chi_map[0.99] = 11.345192964279828;
        chi_map[0.995] = 12.838203077572782;
        chi_map[0.998] = 14.796089301316622;
        chi_map[0.999] = 16.26733195007081;
    }
}

/*Functions for KP estimation using darknet
This part could be placed on another file
*/

template<typename T>
int intRoundUp(const T a) {
    return int(a + 0.5f);
}

template<typename T>
T fastMinimum(const T a, const T b) {
    return (a < b ? a : b);
}

void Detector::render_pose_keypoints(sl::Mat& frame_, std::vector<int> keyshape, const float threshold) {

    cv::Mat frame = slMat2cvMat(frame_);

    const int num_keypoints = keyshape[1];
    ////////////////////////// COCO MODEL ///////////////////////////////
    //unsigned int pairs[] = {
    //    1, 2, 1, 5, 2, 3, 3, 4, 5, 6, 6, 7, 1, 99, 8, 9, 9, 10,
    //    8, 11, 11, 12, 12, 13, 1, 0, 0, 14, 14, 16, 0, 15, 15, 17
    //};
    ////////////////////////// COCO MODEL ///////////////////////////////



    //////////////////////////  BODY_25  ///////////////////////////////
    unsigned int pairs[] = {
        0, 1, 1, 2, 2, 3, 3, 4, 1, 5, 5, 6, 6, 7, 1, 8, 8, 9, 9, 10,
        10, 11, 11, 22, 11, 24, 8, 12, 12, 13, 13, 14, 14, 19, 14, 21,
    };
    //////////////////////////  BODY_25  ///////////////////////////////

    float colors[] = {
        255.f, 0.f, 85.f, 255.f, 0.f, 0.f, 255.f, 85.f, 0.f, 255.f, 170.f, 0.f,
        255.f, 255.f, 0.f, 170.f, 255.f, 0.f, 85.f, 255.f, 0.f, 0.f, 255.f, 0.f,
        0.f, 255.f, 85.f, 0.f, 255.f, 170.f, 0.f, 255.f, 255.f, 0.f, 170.f, 255.f,
        0.f, 85.f, 255.f, 0.f, 0.f, 255.f, 255.f, 0.f, 170.f, 170.f, 0.f, 255.f,
        255.f, 0.f, 255.f, 85.f, 0.f, 255.f
    };
    const int pairs_size = sizeof(pairs) / sizeof(unsigned int);
    const int number_colors = sizeof(colors) / sizeof(float);

    for(int person = 0; person < keyshape[0]; ++person) {
        // Draw lines
        for(int pair = 0u; pair < pairs_size; pair += 2) {
            const int index1 = (person * num_keypoints + pairs[pair]) * keyshape[2];
            const int index2 = (person * num_keypoints + pairs[pair + 1]) * keyshape[2];
            if(keypoints[index1 + 2] > threshold && keypoints[index2 + 2] > threshold) {
                const int color_index = pairs[pair + 1] * 3;
                cv::Scalar color{colors[(color_index + 2) % number_colors],
                    colors[(color_index + 1) % number_colors],
                    colors[(color_index + 0) % number_colors]};

                cv::Point keypoint1{intRoundUp(keypoints[index1] * scale), intRoundUp(keypoints[index1 + 1] * scale)};
                cv::Point keypoint2{intRoundUp(keypoints[index2] * scale), intRoundUp(keypoints[index2 + 1] * scale)};
                cv::line(frame, keypoint1, keypoint2, color, 2);
            }
            //}
        }
        // Draw circles
        for(int part = 0; part < num_keypoints + 1; ++part) {
            const int index = (person * num_keypoints + part) * keyshape[2];
            if(keypoints[index + 2] > threshold) {
                const int color_index = part * 3;
                cv::Scalar color{colors[(color_index + 2) % number_colors],
                    colors[(color_index + 1) % number_colors],
                    colors[(color_index + 0) % number_colors]};
                cv::Point center{intRoundUp(keypoints[index] * scale), intRoundUp(keypoints[index + 1] * scale)};
                cv::circle(frame, center, 2, color, -1);
            }
            // }
        }
    }
}

void Detector::connect_bodyparts(std::vector<float>& pose_keypoints, const float* const map, const float* const peaks, int mapw, int maph, const int inter_min_above_th, const float inter_th, const int min_subset_cnt, const float min_subset_score, std::vector<int>& keypoint_shape) {
    keypoint_shape.resize(3);


    //////////////////////////  COCO MODEL  ///////////////////////////////
    //const int body_part_pairs[] = {
    //    1, 2, 1, 5, 2, 3, 3, 4, 5, 6, 6, 7, 1, 8, 8, 9, 9, 10, 1, 11, 11,
    //    12, 12, 13, 1, 0, 0, 14, 14, 16, 0, 15, 15, 17, 2, 16, 5, 17
    //};
    //
    //const int limb_idx[] = {
    //    31, 32, 39, 40, 33, 34, 35, 36, 41, 42, 43, 44, 19, 20, 21, 22, 23, 24, 25,
    //    26, 27, 28, 29, 30, 47, 48, 49, 50, 53, 54, 51, 52, 55, 56, 37, 38, 45, 46
    //};
    //////////////////////////  COCO MODEL  ///////////////////////////////



    ////////////////////////// BODY_25  MODEL ///////////////////////////////
    const int body_part_pairs[] = {
        1, 8,   //0
        1, 2,   //1
        1, 5,   //2
        2, 3,   //3
        3, 4,   //4
        5, 6,   //5
        6, 7,   //6
        8, 9,   //7
        9, 10,  //8
        10, 11, //9
        8, 12,  //10
        12, 13, //11
        13, 14, //12
        1, 0,   //13
        0, 15,  //14
        15, 17, //15
        0, 16,  //16
        16, 18, //17
        2, 17,  //18
        5, 18,  //19
        14, 19, //20
        19, 20, //21
        14, 21, //22
        11, 22, //23
        22, 23, //24
        11, 24
    }; //25

    const int limb_idx[] = {
        26, 27, //0   Nose
        40, 41, //1   Neck
        48, 49, //2   RShoulder
        42, 43, //3   RElbow
        44, 45, //4   RWrist
        50, 51, //5   LShoulder
        52, 53, //6   LElbow
        32, 33, //7   LWrist
        28, 29, //8   MidHip
        30, 31, //9   RHip
        34, 35, //10  RKnee
        36, 37, //11  RAnkle
        38, 39, //12  LHip
        56, 57, //13  LKnee
        58, 59, //14  LAnkle
        62, 63, //15  REye
        60, 61, //16  LEye
        64, 65, //17  REar
        46, 47, //18  LEar
        54, 55, //19  LBigToe
        66, 67, //20  LSmallToe
        68, 69, //21  LHeel
        70, 71, //22  RBigToe
        72, 73, //23  RSmallToe
        74, 75, //24  RHeel
        76, 77
    }; //25  Background
       ////////////////////////// BODY_25 MODEL ///////////////////////////////


    const int num_body_parts = 25; // COCO part number = 18 // BODY_25 part number = 25

    const int num_body_part_pairs = 26; // 19 body part pairs // BODY_25 = 26
    std::vector<std::pair < std::vector<int>, double>> subset;
    const int subset_counter_index = num_body_parts;
    const int subset_size = num_body_parts + 1;
    const int peaks_offset = 3 * (POSE_MAX_PEOPLE + 1);
    const int map_offset = mapw * maph;

    for(unsigned int pair_index = 0u; pair_index < num_body_part_pairs; ++pair_index) {
        const int body_partA = body_part_pairs[2 * pair_index];
        const int body_partB = body_part_pairs[2 * pair_index + 1];
        const float* candidateA = peaks + body_partA*peaks_offset;
        const float* candidateB = peaks + body_partB*peaks_offset;
        const int nA = (int) (candidateA[0]); // number of part A candidates
        const int nB = (int) (candidateB[0]); // number of part B candidates

                                              // add parts into the subset in special case
        if(nA == 0 || nB == 0) {
            // Change w.r.t. other
            if(nA == 0) // nB == 0 or not
            {
                for(int i = 1; i <= nB; ++i) {
                    bool num = false;
                    for(unsigned int j = 0u; j < subset.size(); ++j) {
                        const int off = body_partB * peaks_offset + i * 3 + 2;
                        if(subset[j].first[body_partB] == off) {
                            num = true;
                            break;
                        }
                    }
                    if(!num) {
                        std::vector<int> row_vector(subset_size, 0);
                        // store the index
                        row_vector[body_partB] = body_partB * peaks_offset + i * 3 + 2;
                        // the parts number of that person
                        row_vector[subset_counter_index] = 1;
                        // total score
                        const float subsetScore = candidateB[i * 3 + 2];
                        subset.emplace_back(std::make_pair(row_vector, subsetScore));
                    }
                }
            } else // if (nA != 0 && nB == 0)
            {
                for(int i = 1; i <= nA; i++) {
                    bool num = false;
                    for(unsigned int j = 0u; j < subset.size(); ++j) {
                        const int off = body_partA * peaks_offset + i * 3 + 2;
                        if(subset[j].first[body_partA] == off) {
                            num = true;
                            break;
                        }
                    }
                    if(!num) {
                        std::vector<int> row_vector(subset_size, 0);
                        // store the index
                        row_vector[body_partA] = body_partA * peaks_offset + i * 3 + 2;
                        // parts number of that person
                        row_vector[subset_counter_index] = 1;
                        // total score
                        const float subsetScore = candidateA[i * 3 + 2];
                        subset.emplace_back(std::make_pair(row_vector, subsetScore));
                    }
                }
            }
        } else // if (nA != 0 && nB != 0)
        {
            std::vector<std::tuple<double, int, int>> temp;
            const int num_inter = 10;
            // limb PAF x-direction heatmap
            const float* const mapX = map + limb_idx[2 * pair_index] * map_offset;
            // limb PAF y-direction heatmap
            const float* const mapY = map + limb_idx[2 * pair_index + 1] * map_offset;
            // start greedy algorithm
            for(int i = 1; i <= nA; i++) {
                for(int j = 1; j <= nB; j++) {
                    const int dX = candidateB[j * 3] - candidateA[i * 3];
                    const int dY = candidateB[j * 3 + 1] - candidateA[i * 3 + 1];
                    const float norm_vec = float(std::sqrt(dX * dX + dY * dY));
                    // If the peaksPtr are coincident. Don't connect them.
                    if(norm_vec > 1e-6) {  //if (norm_vec > 1e-6)
                        const float sX = candidateA[i * 3];
                        const float sY = candidateA[i * 3 + 1];
                        const float vecX = dX / norm_vec;
                        const float vecY = dY / norm_vec;
                        float sum = 0.;
                        int count = 0;
                        for(int lm = 0; lm < num_inter; lm++) {
                            const int mX = fastMinimum(mapw - 1, intRoundUp(sX + lm * dX / num_inter));
                            const int mY = fastMinimum(maph - 1, intRoundUp(sY + lm * dY / num_inter));
                            const int idx = mY * mapw + mX;
                            const float score = (vecX * mapX[idx] + vecY * mapY[idx]);
                            if(score > inter_th) {
                                sum += score;
                                ++count;
                            }
                        }

                        // parts score + connection score
                        if(count > inter_min_above_th) {
                            temp.emplace_back(std::make_tuple(sum / count, i, j));
                        }
                    }
                }
            }
            // select the top minAB connection, assuming that each part occur only once
            // sort rows in descending order based on parts + connection score
            if(!temp.empty()) {
                std::sort(temp.begin(), temp.end(), std::greater<std::tuple<float, int, int>>());
            }
            std::vector<std::tuple<int, int, double>> connectionK;

            const int minAB = fastMinimum(nA, nB);
            // assuming that each part occur only once, filter out same part1 to different part2
            std::vector<int> occurA(nA, 0);
            std::vector<int> occurB(nB, 0);
            int counter = 0;
            for(unsigned int row = 0u; row < temp.size(); row++) {
                const float score = std::get<0>(temp[row]);
                const int aidx = std::get<1>(temp[row]);
                const int bidx = std::get<2>(temp[row]);
                if(!occurA[aidx - 1] && !occurB[bidx - 1]) {
                    // save two part score "position" and limb mean PAF score
                    connectionK.emplace_back(std::make_tuple(body_partA * peaks_offset + aidx * 3 + 2,
                        body_partB * peaks_offset + bidx * 3 + 2, score));
                    ++counter;
                    if(counter == minAB) {
                        break;
                    }
                    occurA[aidx - 1] = 1;
                    occurB[bidx - 1] = 1;
                }
            }
            // Cluster all the body part candidates into subset based on the part connection
            // initialize first body part connection
            if(pair_index == 0) {
                for(const auto connectionKI : connectionK) {
                    std::vector<int> row_vector(num_body_parts + 3, 0);
                    const int indexA = std::get<0>(connectionKI);
                    const int indexB = std::get<1>(connectionKI);
                    const double score = std::get<2>(connectionKI);
                    row_vector[body_part_pairs[0]] = indexA;
                    row_vector[body_part_pairs[1]] = indexB;
                    row_vector[subset_counter_index] = 2;
                    // add the score of parts and the connection
                    const double subset_score = peaks[indexA] + peaks[indexB] + score;
                    subset.emplace_back(std::make_pair(row_vector, subset_score));
                }
            }// Add ears connections (in case person is looking to opposite direction to camera)
            else if(pair_index == 19 || pair_index == 18) { // COCO = 17 || 18 // BODY_25 = 18 || 19
                for(const auto& connectionKI : connectionK) {
                    const int indexA = std::get<0>(connectionKI);
                    const int indexB = std::get<1>(connectionKI);
                    for(auto& subsetJ : subset) {
                        auto& subsetJ_first = subsetJ.first[body_partA];
                        auto& subsetJ_first_plus1 = subsetJ.first[body_partB];
                        if(subsetJ_first == indexA && subsetJ_first_plus1 == 0) {
                            subsetJ_first_plus1 = indexB;
                        } else if(subsetJ_first_plus1 == indexB && subsetJ_first == 0) {
                            subsetJ_first = indexA;
                        }
                    }
                }
            } else {
                if(!connectionK.empty()) {
                    for(unsigned int i = 0u; i < connectionK.size(); ++i) {
                        const int indexA = std::get<0>(connectionK[i]);
                        const int indexB = std::get<1>(connectionK[i]);
                        const double score = std::get<2>(connectionK[i]);
                        int num = 0;
                        // if A is already in the subset, add B
                        for(unsigned int j = 0u; j < subset.size(); j++) {
                            if(subset[j].first[body_partA] == indexA) {
                                subset[j].first[body_partB] = indexB;
                                ++num;
                                subset[j].first[subset_counter_index] = subset[j].first[subset_counter_index] + 1;
                                subset[j].second = subset[j].second + peaks[indexB] + score;
                            }
                        }
                        // if A is not found in the subset, create new one and add both
                        if(num == 0) {
                            std::vector<int> row_vector(subset_size, 0);
                            row_vector[body_partA] = indexA;
                            row_vector[body_partB] = indexB;
                            row_vector[subset_counter_index] = 2;
                            const float subsetScore = peaks[indexA] + peaks[indexB] + score;
                            subset.emplace_back(std::make_pair(row_vector, subsetScore));
                        }
                    }
                }
            }
        }
    }

    // Delete people below thresholds, and save to output
    int number_people = 0;
    std::vector<int> valid_subset_indexes;
    valid_subset_indexes.reserve(fastMinimum((size_t) POSE_MAX_PEOPLE, subset.size()));
    for(unsigned int index = 0; index < subset.size(); ++index) {
        const int subset_counter = subset[index].first[subset_counter_index];
        const double subset_score = subset[index].second;
        if(subset_counter >= min_subset_cnt && (subset_score / subset_counter) > min_subset_score) {
            ++number_people;
            valid_subset_indexes.emplace_back(index);
            if(number_people == POSE_MAX_PEOPLE) {
                break;
            }
        }
    }

    // Fill and return pose_keypoints
    keypoint_shape = {number_people, (int) num_body_parts, 3};
    if(number_people > 0) {
        pose_keypoints.resize(number_people * (int) num_body_parts * 3);
    } else {
        pose_keypoints.clear();
    }
    for(unsigned int person = 0u; person < valid_subset_indexes.size(); ++person) {
        const auto& subsetI = subset[valid_subset_indexes[person]].first;
        for(int bodyPart = 0u; bodyPart < num_body_parts; bodyPart++) {
            const int base_offset = (person * num_body_parts + bodyPart) * 3;
            const int body_part_index = subsetI[bodyPart];
            if(body_part_index > 0) {
                pose_keypoints[base_offset] = peaks[body_part_index - 2];
                pose_keypoints[base_offset + 1] = peaks[body_part_index - 1];
                pose_keypoints[base_offset + 2] = peaks[body_part_index];
            } else {
                pose_keypoints[base_offset] = 0.f;
                pose_keypoints[base_offset + 1] = 0.f;
                pose_keypoints[base_offset + 2] = 0.f;
            }
        }
    }
    //std::cout << number_people << std::endl;
    keypoints = pose_keypoints;
}

std::vector<sl::float4> Detector::estimate_keypoints_depth(std::vector<float> poseKeypoints) {
    const auto numberPeopleDetected = shape[0];
    const auto numberBodyParts = shape[1];

    //For PAF searching, finding limbs. Represents every connection of every keypoint
    std::map<int, std::vector<int>> limb_map
    {
        {0,{1, 15, 16}},
        {1,{0, 2, 5, 8}},
        {2,{1, 3}},
        {3,{2, 4}},
        {4,{3}},
        {5,{1, 6}},
        {6,{5, 7}},
        {7,{6}},
        {8,{1, 9, 12}},
        {9,{8, 10}},
        {10,{9, 11}},
        {11,{10, 22, 23, 24}},
        {12,{8, 13}},
        {13,{12, 14}},
        {14,{13, 19, 20, 21}},
        {15,{0, 17}},
        {16,{0, 18}},
        {17,{15}},
        {18,{16}},
        {19,{14, 20}},
        {20,{19}},
        {21,{14}},
        {22,{11, 23}},
        {23,{22}},
        {24,{11}},
    };

    //std::cout << shape[0] << "," << shape[1] << "," << shape[2] << std::endl;

    //For PAF searching, passing limb position
    std::vector<std::pair<int, int>> limb_position;
    int i, j;

    // Function output, representing x, y and z coordinates in the 
    // global coordinate frame + confidence
    std::vector<sl::float4> depth_keypoints;

    // For each detected person
    for(int person = 0; person < numberPeopleDetected; person++) {

        std::map<int, sl::float4> keypoints_position; // 3D + score for each keypoints
        float score;

        // Iterate over their keypoints
        for(int k = 0; k < numberBodyParts; k++) {
            score = poseKeypoints[(person * numberBodyParts + k) * shape[2] + 2];
            keypoints_position[k] = sl::float4(NAN, NAN, NAN, score);

            // By default the network will throw a kp with 0 confidence if it was not found
            if(score > 0) {

                i = (int)(poseKeypoints[(person * numberBodyParts + k) * shape[2]] * scale);
                j = (int)(poseKeypoints[(person * numberBodyParts + k) * shape[2] + 1] * scale);

                if (i != 0 || j != 0)
                {
                    //  sl::ERROR_CODE val = xyz.getValue<sl::float4>(i, j, &keypoints_position[k], sl::MEM_CPU);
                    bool val = getXYZWorld(i, j, keypoints_position[k]);

                    if(!std::isfinite(keypoints_position[k].z) or !val/* == sl::ERROR_CODE_FAILURE*/) {
                        limb_position.clear();
                        for(int h = 0; h < limb_map[k].size(); h++) {
                            int i_pair = round(poseKeypoints[(person * numberBodyParts + limb_map[k][h]) * shape[2]] * scale);
                            int j_pair = round(poseKeypoints[(person * numberBodyParts + limb_map[k][h]) * shape[2] + 1] * scale);
                            std::pair<int, int> aux(i_pair, j_pair);
                            if (i_pair != 0 && j_pair != 0)
                                limb_position.push_back(aux);
                        }
                        //printf("bodypart %d\n", k);
                        keypoints_position[k] = PAF_getPatchIdx((const int) i, (const int) j, limb_position);
                    }
                }

                if(i == 0 && j == 0) {
                    keypoints_position[k] = sl::float4(NAN, NAN, NAN, 0);
                    score = 0;
                }
            }
            sl::float4 coord_world_frame(keypoints_position[k].x, keypoints_position[k].y, keypoints_position[k].z, score);
            depth_keypoints.push_back(coord_world_frame);
        }
    }
    return depth_keypoints;
}

void Detector::find_heatmap_peaks(const float *src, float *dst, const int SRCW, const int SRCH, const int SRC_CH, const float TH, int s_w, int s_h) {
    // find peaks (8-connected neighbor), weights with 7 by 7 area to get sub-pixel location and response
    const int SRC_PLANE_OFFSET = SRCW * SRCH;
    // add 1 for saving total people count, 3 for x, y, score
    const int DST_PLANE_OFFSET = (POSE_MAX_PEOPLE + 1) * 3;
    float *dstptr = dst;
    int c = 0;
    int x = 0;
    int y = 0;
    int i = 0;
    int j = 0;
    // TODO: reduce multiplication by using pointer
    for(c = 0; c < SRC_CH - 1; ++c) {
        int num_people = 0;
        for(y = 1; y < s_h - 1 && num_people != POSE_MAX_PEOPLE; ++y) {
            for(x = 1; x < s_w - 1 && num_people != POSE_MAX_PEOPLE; ++x) {
                int idx = y * SRCW + x;
                float value = src[idx];
                if(value > TH) {
                    const float TOPLEFT = src[idx - SRCW - 1];
                    const float TOP = src[idx - SRCW];
                    const float TOPRIGHT = src[idx - SRCW + 1];
                    const float LEFT = src[idx - 1];
                    const float RIGHT = src[idx + 1];
                    const float BUTTOMLEFT = src[idx + SRCW - 1];
                    const float BUTTOM = src[idx + SRCW];
                    const float BUTTOMRIGHT = src[idx + SRCW + 1];
                    if(value > TOPLEFT && value > TOP && value > TOPRIGHT && value > LEFT &&
                        value > RIGHT && value > BUTTOMLEFT && value > BUTTOM && value > BUTTOMRIGHT) {
                        float x_acc = 0;
                        float y_acc = 0;
                        float score_acc = 0;
                        for(i = -3; i <= 3; ++i) {
                            int ux = x + i;
                            if(ux >= 0 && ux < s_w) {
                                //if (ux >= 0 && ux < SRCW) {
                                for(j = -3; j <= 3; ++j) {
                                    int uy = y + j;
                                    if(uy >= 0 && uy < s_h) {
                                        //if (uy >= 0 && uy < SRCH) {
                                        float score = src[uy * SRCW + ux];
                                        x_acc += ux * score;
                                        y_acc += uy * score;
                                        if(score <= 1)
                                            score_acc += score;
                                        else
                                            score_acc += 1.0;
                                    }
                                }
                            }
                        }
                        x_acc /= score_acc;
                        y_acc /= score_acc;
                        score_acc = value;
                        dstptr[(num_people + 1) * 3 + 0] = x_acc;
                        dstptr[(num_people + 1) * 3 + 1] = y_acc;
                        dstptr[(num_people + 1) * 3 + 2] = score_acc;
                        ++num_people;
                    }
                }
            }
        }
        dstptr[0] = num_people;
        src += SRC_PLANE_OFFSET;
        dstptr += DST_PLANE_OFFSET;
    }
}

cv::Mat Detector::create_netsize_im(const cv::Mat &im, const int netw, const int neth, float *scale, float *s_w, float *s_h) {
    // for tall image
    int newh = neth;
    float s = newh / (float) im.rows;
    int neww = im.cols * s;
    if(neww > netw) {
        //for fat image
        neww = netw;
        s = neww / (float) im.cols;
        newh = im.rows * s;
    }
    *scale = 1 / s;
    cv::Rect dst_area(0, 0, neww, newh);
    cv::Mat dst = cv::Mat::zeros(neth, netw, CV_8UC3);
    cv::resize(im, dst(dst_area), cv::Size(neww, newh));
    *s_w = neww;
    *s_h = newh;
    return dst;
}

inline sl::float4 warpPoint(sl::float4 &pt_curr, sl::Matrix4f &path) {
    sl::float4 proj3D;
    float* p_path = path.m;
    proj3D.x = pt_curr.x * p_path[0] + pt_curr.y * p_path[1] + pt_curr.z * p_path[2] + p_path[3];
    proj3D.y = pt_curr.x * p_path[4] + pt_curr.y * p_path[5] + pt_curr.z * p_path[6] + p_path[7];
    proj3D.z = pt_curr.x * p_path[8] + pt_curr.y * p_path[9] + pt_curr.z * p_path[10] + p_path[11];
    return proj3D;
}

bool /*valid*/ Detector::getXYZWorld(int i, int j, sl::float4 &value) {
    float depth;

    if((i >= 0) && (i < currentDepth.getWidth()) && (j >= 0) && (j < currentDepth.getHeight())) {
        currentDepth.getValue<float>(i, j, &depth, sl::MEM_CPU);
        if(!std::isfinite(depth)) return false;

        depth *= depth_scale; // mm -> m

        value.z = depth;
        value.x = (i - cx) * depth * fx_;
        value.y = (j - cy) * depth * fy_;
        value.w = 1;
        value = warpPoint(value, currentPose);
        //   printf("Value %f %f %f\n", value.x, value.y, value.z);
        return true;
    } else return false;
}

// The 3D of the point is not directly taken 'as is'. If the measurement isn't valid, we look around the point in 2D to find a close point with a valid depth

sl::float4 Detector::PAF_getPatchIdx(const int &center_i, const int &center_j, std::vector<std::pair<int, int>> limb_direction) {
    sl::float4 out(NAN, NAN, NAN, 0);
    int i, j;
    const int R_max = 7; //10
    if (limb_direction.size() > 0)
    {
        //printf("got into PAF search\n");
        int max_iterations = 1;
        std::map<int, std::pair<double, double>> offset;
        for(int a = 0; a < limb_direction.size(); a++) {
            double dist = std::sqrt((center_i - limb_direction[a].first)*(center_i - limb_direction[a].first) + (center_j - limb_direction[a].second)*(center_j - limb_direction[a].second));
            int it = (int)(dist / (2 * R_max));
            if(it > max_iterations)
                max_iterations = it;
    
            // x/y ratio
            double arcsin = std::asin(std::abs(center_j - limb_direction[a].second) / dist);
    
            int i_signal = center_i - limb_direction[a].first <= 0 ? 1 : -1;
            int j_signal = center_j - limb_direction[a].second <= 0 ? 1 : -1;
            offset[a] = {i_signal * 2 * R_max * std::cos(arcsin), j_signal * 2 * R_max * std::sin(arcsin)};
        }
        int i_it(center_i), j_it(center_j);
    
        for(int count = 0; count < max_iterations; count++) {
            for(int limb = 0; limb < limb_direction.size(); limb++) {
                i_it = center_i + (int)(count * offset[limb].first);
                j_it = center_j + (int)(count * offset[limb].second);

                bool go = true;
                //printf("offset.x = %lf      offset.y = %lf\n", offset[limb].first, offset[limb].second);
                //printf("i_it : %d - limb_direction : %d\nj_it : %d - limb_direction : %d\n", i_it, limb_direction[limb].first, j_it, limb_direction[limb].second);
                // Check valid beginning in x
                if(i_it >= 0)
                {
                    // If the next limb has a bigger x than this limb
                    if (offset[limb].first >= 0)
                    {
                        if(i_it > limb_direction[limb].first)
                            go = false;
                    }
                    else
                    {
                        if(i_it < limb_direction[limb].first)
                            go = false;
                    }
                }else go = false;

                // Check valid beginning in y
                if(j_it >= 0)
                {
                    // If the next limb has a bigger y than this limb
                    if (offset[limb].second >= 0)
                    {
                        if(j_it > limb_direction[limb].second)
                            go = false;
                    }
                    else
                    {
                        if(j_it < limb_direction[limb].second)
                            go = false;
                    }
                }else go = false;

                if(go) {
                    for(int R = 0; R <= R_max; R++) {
                        for(int y = -R; y <= R; y++) {
                                j = j_it + y;
                            for(int x = -R; x <= R && (std::abs(x) == std::abs(y) || R == std::abs(y)); x++) {
                                //printf("entrou\n");
                                i = i_it + x;
                                if(getXYZWorld(i, j, out)){
                                    return out;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        for(int y = -R_max; y <= R_max; y++) {
            j = center_j + y;
            for(int x = -R_max; x <= R_max && (std::abs(x) == std::abs(y) || R_max == std::abs(y)); x++) {
                i = center_i + x;
                if(getXYZWorld(i, j, out)){
                    return out;
                }
            }
        }
    }
    //printf("---------- Did not find depth! -----------\n");
    out.x = NAN;
    out.y = NAN;
    out.z = NAN;
    out.w = 0;
    return out;
}

bool Detector::detectAndTrack(sl::Mat img, sl::Mat depth, timeval time, sl::Transform pose) {
    if(!getIsInit()) {
        //std::cerr << "init(std::string path_cfg, std::string path_model) should be called before detectAndTrack(sl::Mat img, sl::Mat depth, sl::Transform pose, sl::Plane floorPlane)." << std::endl;
        return false;
    }
    currentDepth = depth;
    currentPose = pose;
    current_ts = time.tv_sec * 1000000000 + time.tv_usec * 1000;

    // 1) Create a normalized netsize image and split its channels
    cv::Mat cv_img_rgba = slMat2cvMat(img);
    cv::Mat cv_img;
    cv::cvtColor(cv_img_rgba, cv_img, 1/*CV_RGBA2RGB*/);
    cv::Mat netim;
    netim = create_netsize_im(cv_img, net_inw, net_inh, &scale, &s_w, &s_h);

    netim.convertTo(netim, CV_32F, 1 / 256.f, -0.5);
    float *netin_data_ptr = p_netin_data;
    std::vector<cv::Mat> input_channels;
    for(unsigned i = 0; i < 3; ++i) {
        cv::Mat channel(net_inh, net_inw, CV_32FC1, netin_data_ptr);
        input_channels.emplace_back(channel);
        netin_data_ptr += (net_inw * net_inh);
    }
    cv::split(netim, input_channels);

    // 2) Forward pass the img through the net
    float *netoutdata = runNet(p_netin_data);

    // 3) Resize net output back to input size
    for(unsigned i = 0; i < NET_OUT_CHANNELS; i++) {
        cv::Mat netout(net_outh, net_outw, CV_32F, (netoutdata + net_outh * net_outw * i));
        cv::Mat nmsin(net_inh, net_inw, CV_32F, p_heatmap + net_inh * net_inw * i);
        cv::resize(netout, nmsin, cv::Size(net_inw, net_inh), 0, 0, 2); //2 = CV_INTER_CUBIC
    }

    // 4) Get heatmap peaks
    find_heatmap_peaks(p_heatmap, p_heatmap_peaks, net_inw, net_inh, NET_OUT_CHANNELS, 0.05, round(s_w), round(s_h));

    // 5) Associate keypoints -> outputs a series of 2D keypoints + confidence
    connect_bodyparts(keypoints, p_heatmap, p_heatmap_peaks, net_inw, net_inh, 9, 0.05, 8, 0.4, shape);
    // 9, 0.05, 6, 0.4

#if 0
    printf("\n");
    for (unsigned i = 0; i < shape[0]; i++)
    {
        for (unsigned j = 0; j < 25; j++)
            printf("%d (%f, %f)   ", j, keypoints[75*i + 3*j]*scale, keypoints[75*i + 3*j+1]*scale);
        printf("\n");
    }
    printf("\n");
#endif

    //std::cout << shape[0] << " - ";

    for (unsigned i = 0; i<25; i++)
    {
        printf("%d (%f, %f)", i, keypoints[2*i], keypoints[2*i+1]);
    }
    printf("\n");

    // 6) Estimate keypoints depth -> outputs a series of 3D keypoints + confidence
    depth_keypoints = estimate_keypoints_depth(keypoints);

    // 7) Update the tracker on the newly calculated 3D keypoints
    tracker->newFrame(depth_keypoints, shape, time);
    tracker->updateTracks();

    return true;
}

std::vector<struct zed_tracking::PeopleSkeletonOutput> Detector::getResults() {
    auto out = tracker->getTrackedPeople();
    for(auto &it : out) it.ts = current_ts;
    return out;
}

sl::float3 Detector::getGazeDirection(int id) {
    return tracker->getGazeDirection(id);
}

void Detector::initNet(std::string path_cfg, std::string path_model) {
    aiLoader.init_net(instance_id, (char*) path_cfg.c_str(), (char*) path_model.c_str(), &net_inw, &net_inh, &net_outw, &net_outh);
}

void Detector::initTracker() {
    // Setting tracking parameters
    std::map<double, double> chi_map;
    fillChiMap(chi_map, velocity_in_motion_term);
    double period = 1.0 / rate;
    double gate_distance = chi_map.find(gate_distance_probability) != chi_map.end() ? chi_map[gate_distance_probability] : chi_map[0.999];
    double position_variance = position_variance_weight * std::pow(2 * voxel_size, 2) / 12;
    std::vector<double> likelihood_weights;
    likelihood_weights.push_back(detector_weight * chi_map[0.999] / 18.467);
    likelihood_weights.push_back(motion_weight);

    tracker = new zed_tracking::PersonTracker(gate_distance,
        detector_likelihood,
        likelihood_weights,
        velocity_in_motion_term,
        min_confidence_initialization,
        min_confidence_detections,
        sec_before_old,
        sec_before_fake,
        sec_remain_new,
        detections_to_validate,
        period,
        position_variance,
        acceleration_variance,
        "world",
        false,
        false);
}

float* Detector::runNet(float *indata) {
    float *netoutdata = 0;
    aiLoader.run_net(instance_id, indata, &netoutdata);
    return netoutdata;
}

Detector::Detector() {

}

Detector::~Detector() {
    close();
}

void Detector::init(int instance_id_, std::string path_cfg, std::string path_model) {
    instance_id = instance_id_;
    aiLoader.loadLib();

    bool files_ok = Detector::checkFile(path_cfg);
    files_ok = files_ok && Detector::checkFile(path_model);

    if(files_ok) initNet(path_cfg, path_model);
    else return;

    initTracker();
    allocateMemory();
    isInit = true;
}

void createFolder(const std::string &name) {
#ifdef _WIN32
    CreateDirectory(name.c_str(), NULL);
#else
    mkdir(name.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
}

void Detector::init_v2(int instance_id_, unsigned int net_width, unsigned int net_height, std::string path_compressed_model) {
    instance_id = instance_id_;

    int error_loading = aiLoader.loadLib();
    if(error_loading != 0) return;

    bool files_ok = Detector::checkFile(path_compressed_model);
    if(files_ok) {
        // Uncompress the files
#if 0
        std::string cmd_line = CMD_FOLDER_CREATION + TMP_MODEL_PATH;
        if(system(cmd_line.c_str()) == -1)
            printf("Error creating directory!n");
#else
        createFolder(TMP_MODEL_PATH);
#endif

        path_cfg = TMP_MODEL_PATH + "rYeetQuRW8rfrqV";
        path_model = TMP_MODEL_PATH + "q2Untg346P38dCB";

        deflateModel(path_compressed_model, path_cfg, path_model);

        // Write the size in the conf file 
        std::ifstream filein(path_cfg); //File to read from
        std::string tmp_file_name = path_cfg + "tmp";
        std::ofstream fileout(tmp_file_name); //Temporary file

        if(!filein || !fileout) {
            cout << "Error opening files!" << endl;
            return;
        }

        std::string strTemp, s2 = "width=", s3 = "height=";
        bool found1 = false, found2 = false;
        while(filein >> strTemp) {
            if(strTemp.find(s2) != std::string::npos) {
                strTemp = (s2 + to_string(net_width));
                found1 = true;
            }
            if(strTemp.find(s3) != std::string::npos) {
                strTemp = (s3 + to_string(net_height));
                found2 = true;
            }
            strTemp += "\n";
            fileout << strTemp;
        }

        if((!found1) || (!found2))
            std::cout << "Error write new size in AI model\n";
        filein.close();
        fileout.close();

#if 0
        cmd_line = CMD_RENAME + tmp_file_name + " " + path_cfg;
        system(cmd_line.c_str());
#else
        std::string toto_str;
        toto_str = TMP_MODEL_PATH + "toto";
        if(std::rename(path_cfg.c_str(), toto_str.c_str()))
            cout << "Error rename "<< path_cfg<<" to "<< toto_str <<"\n";
        if(std::rename(tmp_file_name.c_str(), path_cfg.c_str()))
            cout << "Error rename " << tmp_file_name << " to " << path_cfg << "\n";
        if(std::remove(toto_str.c_str()))
            cout << "Error remove "<< toto_str<<"\n";
#endif
    } else return;

    files_ok = Detector::checkFile(path_cfg);
    files_ok = files_ok && Detector::checkFile(path_model);

    if(files_ok) {
        // Load the net
        initNet(path_cfg, path_model);
    } else return;

    initTracker();
    allocateMemory();
    isInit = true;
}

void Detector::close() {
    if(isInit) {
        releaseMemory();
        aiLoader.free_net(instance_id); // free_network_(net);
        isInit = false;

        // Remove the tmp uncompressed files (model and cfg)
        if(!path_cfg.empty() && !path_model.empty()) {
            removeFile(path_cfg);
            removeFile(path_model);
        }
    }
}

void Detector::removeFile(std::string path) {
    /*if (*/remove(path.c_str())/* != 0) perror("Error deleting file")*/;
    //else puts("File successfully deleted");
}

bool Detector::checkFile(std::string path) {
    std::ifstream f(path.c_str());
    return f.good();
}

void Detector::allocateMemory() {
    p_netin_data = new float[net_inw * net_inh * 3]();
    p_heatmap = new float[net_inw * net_inh * NET_OUT_CHANNELS];
    p_heatmap_peaks = new float[3 * (POSE_MAX_PEOPLE + 1) * (NET_OUT_CHANNELS - 1)];
}

void Detector::releaseMemory() {
    if(!p_netin_data) delete[] p_netin_data;
    if(!p_heatmap) delete[] p_heatmap;
    if(!p_heatmap_peaks) delete[] p_heatmap_peaks;
}

unsigned int Detector::generateInstanceID(std::string camera_serial, std::string model_name) {
    /* std::hash<std::string> hasher;
    std::string str(camera_serial + model_name);
    auto hashed = hasher(str); //returns std::size_t
    return (unsigned int) hashed;*/
    int serial_tmp = stoi(camera_serial);
    auto it = serial_to_id.find(serial_tmp);
    if(it == serial_to_id.end()) {
        it = serial_to_id.insert(std::make_pair(serial_tmp, counter_global++)).first;
    }
    return it->second; // serial_to_id[serial_tmp].second;
}

#ifdef WIN32
#define SECURE_OPEN 0
#else
#define SECURE_OPEN 0
#endif

bool readFile(const char* fileName, char **buffer_, long *lSize_, const char *fopen_opt) {
    FILE *fp;
    long lSize;
    char *buffer;
#if SECURE_OPEN
    fopen_s(&fp, fileName, fopen_opt);
#else
    fp = fopen(fileName, fopen_opt);
#endif
    if(!fp) perror(fileName), exit(1);

    fseek(fp, 0L, SEEK_END);
    lSize = ftell(fp);
    rewind(fp);
    //cout << "Read File: " << fileName << " SIZE; " << lSize << "\n";

    /* allocate memory for entire content */
    buffer = (char*) calloc(1, lSize + 1);
    if(!buffer) {
        fclose(fp);
        //fputs("memory alloc fails", stderr);
        return 1;
    }

    /* copy the file into the buffer */
    if(1 != fread(buffer, lSize, 1, fp)) {
        fclose(fp);
        free(buffer);
        //fputs("entire read fails", stderr);
        return 1;
    }

    /* do your work here, buffer is a string contains the whole text */
    fclose(fp);

    *lSize_ = lSize;
    *buffer_ = buffer;

    return 0;
}

int inflate_(const char * file, FILE *fp, const char *fopen_opt) {
    char *buffer;
    long size;
    bool status = readFile(file, &buffer, &size, fopen_opt);
    if(status) return 1;

    const unsigned char *input = (const unsigned char*) buffer;

    int cmp_status;
    uLong src_len = (uLong) size;
    uLong cmp_len = compressBound(src_len);
    uint8_t *pCmp;

    // Allocate buffers to hold compressed and uncompressed data.
    pCmp = (mz_uint8 *) malloc((size_t) cmp_len);
    if(!pCmp) return 2;

    // Compress the string.
    cmp_status = compress(pCmp, &cmp_len, input, src_len);
    if(cmp_status != Z_OK) {
        free(pCmp);
        return 3;
    }

    // WRITE :
    // size of string
    uLong size_cmp[2];
    size_cmp[0] = cmp_len;
    size_cmp[1] = src_len;
    fwrite(size_cmp, sizeof(uLong) * 2, 1, fp);
    // actual compressed string
    fwrite(pCmp, cmp_len, 1, fp);
    free(buffer);
    free(pCmp);
    return 0;
}

bool Detector::inflateModel(std::string &archive, std::string path_cfg, std::string path_model) {

    // --------------------------------------------- OPEN ARCHIVE -----------------------------
    // Open the archive
    FILE *fp;
#if SECURE_OPEN
    fopen_s(&fp, archive.c_str(), "wb");
#else
    fp = fopen(archive.c_str(), "wb");
#endif
    if(fp == NULL) {
        //std::cout << "Error open Model\n";
        return 1;
    }

    // --------------------------------------------- CFG FILE -----------------------------
    // Open cfg file and put it in a string
    int err_infl = inflate_(path_cfg.c_str(), fp, "rb");
    if(err_infl) {
        //cout << "Error inflate CFG " << err_infl << "\n";
        return 1;
    }
    // --------------------------------------------- Weight FILE -----------------------------
    // Open weight file and put it in a string
    err_infl = inflate_(path_model.c_str(), fp, "rb");
    if(err_infl) {
        //cout << "Error inflate MODEL " << err_infl << "\n";
        return 1;
    }
    // --------------------------------------------- CLOSE ARCHIVE -----------------------------
    fclose(fp);
    return 0;
}

int deflate_(const char *file, FILE *fp, const char *fopen_opt) {
    int cmp_status;
    uLong cmp_len;
    uint8_t *pUncomp, *pCmp;

    // Read yml compressed string size
    uLong size_cmp[2];
    fread(size_cmp, sizeof(uLong) * 2, 1, fp);
    cmp_len = size_cmp[0];
    pCmp = (mz_uint8 *) malloc((size_t) cmp_len);
    uLong uncomp_len = size_cmp[1];
    pUncomp = (mz_uint8 *) malloc((size_t) uncomp_len);
    if((!pCmp) || (!pUncomp)) return 1; // out of memory
                                        // Read yml compressed
    fread(pCmp, cmp_len, 1, fp);
    //cout << "deflate_ File: " << file << " SIZE " << cmp_len << " length " << uncomp_len << "\n";
    // Decompress.
    cmp_status = uncompress(pUncomp, &uncomp_len, pCmp, cmp_len);
    if(cmp_status != Z_OK) {
        free(pUncomp);
        return 2;
    }
    free(pCmp);

    // Flush the text into a file
#if SECURE_OPEN
    FILE *f_tmp;
    fopen_s(&f_tmp, file, fopen_opt);
#else
    FILE *f_tmp = fopen(file, fopen_opt);
#endif
    if(f_tmp == NULL) return 3;
    fwrite(pUncomp, sizeof(char), uncomp_len, f_tmp);
    fclose(f_tmp);
    free(pUncomp);
    return 0;
}

bool Detector::deflateModel(std::string &archive, std::string path_cfg, std::string path_model) {

    // --------------------------------------------- OPEN ARCHIVE -----------------------------
    // Open the archive
    FILE *fp;
#if SECURE_OPEN
    fopen_s(&fp, archive.c_str(), "rb");
#else
    fp = fopen(archive.c_str(), "rb");
#endif
    if(fp == NULL) return 1;

    int err_difl = deflate_(path_cfg.c_str(), fp, "wb");
    // --------------------------------------------- CFG FILE -----------------------------
    if(err_difl) {
        //cout << "Error deflate CFG " << err_difl << "\n";
        return 1;
    }
    // --------------------------------------------- Weight FILE -----------------------------
    // Read yml compressed string size
    err_difl = deflate_(path_model.c_str(), fp, "wb");
    if(err_difl) {
        //cout << "Error deflate MODEL " << err_difl << "\n";
        return 1;
    }
    // --------------------------------------------- CLOSE ARCHIVE -----------------------------
    fclose(fp);
    return 0;
}

void Detector::setCalibParams(float &fx, float &fy, float &cx_, float &cy_, float unit_scale) {
    fx_ = 1 / fx;
    fy_ = 1 / fy;
    cx = cx_;
    cy = cy_;
    depth_scale = unit_scale;
}
