#include "sl_core/ai/skeleton/tracking/person_tracker.h"

#define TRY_NEW_BARYCENTER

namespace zed_tracking {

    int PersonTracker::count = 0;

    PersonTracker::~PersonTracker() {
        PersonTracker::count++;
    }

    void PersonTracker::newFrame(const std::vector<sl::float4>& depth_joints, std::vector<int> shape, struct timeval time) {
        int n_people = shape[0];
        int n_bodyparts = shape[1];
        int n_data = shape[2];

        detections_.clear();
        unassociated_detections_.clear();
        lost_tracks_.clear();
        new_tracks_.clear();

        struct timeval currentTime;

        if (n_people > 0)
            currentTime = time;
        else
            gettimeofday(&currentTime, NULL);

        // create and assign the detections vector
        std::vector<zed_tracking::Detection> detections;

        for (unsigned i = 0; i < n_people; i++) {
            std::vector<Eigen::Vector4d> world_ref_keypoints;
            sl::float4 center_gravity(0, 0, 0, 0);

#ifndef TRY_NEW_BARYCENTER
            int count = 0;

            //let's try not considering feet for barycenter computation
            for (unsigned k = 0; k < n_bodyparts; k++) {
                Eigen::Vector4d kps(depth_joints[(i * n_bodyparts + k)].x,
                        depth_joints[(i * n_bodyparts + k)].y,
                        depth_joints[(i * n_bodyparts + k)].z,
                        depth_joints[(i * n_bodyparts + k)].w);

                //if (k!=15 && k!=16 && k!=17 && k!=18 && k!=22 && k!=23 && k!=24 && k!=19 && k!=20 && k!=21)
                if (k!=22 && k!=23 && k!=24 && k!=19 && k!=20 && k!=21)
                {
                    if (std::isfinite(kps(2)) && kps(3) > .5) {
                        center_gravity += depth_joints[(i * n_bodyparts + k)];
                        count++;
                    }
                }

                world_ref_keypoints.push_back(kps);

            }

            if (count != 0) {
                center_gravity.x /= (float) count;
                center_gravity.y /= (float) count;
                center_gravity.z /= (float) count;

                Eigen::Vector3d barycenter(center_gravity.x, center_gravity.y, center_gravity.z);
                zed_tracking::Detection det(world_ref_keypoints,
                        barycenter,
                        currentTime,
                        (double) (barycenter(2)));
                        //(double) (barycenter.norm()));
                //std::cout << det.getConfidence() << ", ";
                if (det.getConfidence() > 0.0)
                    detections.push_back(det);
            }
#else
            std::vector<float> x_value;
            std::vector<float> y_value;
            std::vector<float> z_value;

            x_value.reserve(n_bodyparts);
            y_value.reserve(n_bodyparts);
            z_value.reserve(n_bodyparts);
            //int j = 0;
            for (int k = 0; k < n_bodyparts; k++)
            {
                Eigen::Vector4d kps(depth_joints[(i * n_bodyparts + k)].x,
                        depth_joints[(i * n_bodyparts + k)].y,
                        depth_joints[(i * n_bodyparts + k)].z,
                        depth_joints[(i * n_bodyparts + k)].w);

                float v_x = depth_joints[(i * n_bodyparts + k)].x;
                float v_y = depth_joints[(i * n_bodyparts + k)].y;
                float v_z = depth_joints[(i * n_bodyparts + k)].z;
                float v_w = depth_joints[(i * n_bodyparts + k)].w;
                //if (v_x !=0.f || v_y != 0.f || v_z != 0.f)
                if (std::isfinite(v_x) || std::isfinite(v_y) || std::isfinite(v_z))
                {
                    x_value.push_back(v_x);
                    y_value.push_back(v_y);
                    z_value.push_back(v_z);
                }
                world_ref_keypoints.push_back(kps);

            }
            int middle = (x_value.size()) / 2;

            std::nth_element(x_value.begin(), x_value.begin() + middle, x_value.end());
            std::nth_element(y_value.begin(), y_value.begin() + middle, y_value.end());
            std::nth_element(z_value.begin(), z_value.begin() + middle, z_value.end());

            float x_median = x_value[middle];
            float y_median = y_value[middle];
            float z_median = z_value[middle];

            // ------------------------------------------------------

            std::vector<float> dist;
            dist.reserve(n_bodyparts);
            std::vector<int> index;
            index.reserve(n_bodyparts);
            std::vector<float> dist_info;
            dist_info.reserve(n_bodyparts);

            float d_max = -1e9f;
            float d_min = 1e9f;
            for (int k = 0; k < n_bodyparts; k++)
            {
                float v_x = depth_joints[(i * n_bodyparts + k)].x;
                float v_y = depth_joints[(i * n_bodyparts + k)].y;
                float v_z = depth_joints[(i * n_bodyparts + k)].z;
                //if (v_x != 0.f || v_y != 0.f || v_z != 0.f)
                if (std::isfinite(v_x) || std::isfinite(v_y) || std::isfinite(v_z))
                {
                    float dist_x = (v_x - x_median);
                    float dist_y = (v_y - y_median);
                    float dist_z = (v_z - z_median);

                    float di = dist_x*dist_x + dist_y*dist_y + dist_z*dist_z;

                    dist.push_back(di);
                    index.push_back(k);
                    dist_info.push_back(di);

                    if (di < d_min)
                        d_min = di;

                    if (di > d_max)
                        d_max = di;

                    //printf("joint %d  => distance from bary : %f \n", i, dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
                }

            }

            int len = dist.size();
            int mid = len / 2;

            std::nth_element(dist.begin(), dist.begin() + mid, dist.end());

            float median_dist = dist[mid];

            int valid_size = dist_info.size();

            float i_median_dist = 1.f / median_dist;

            x_value.clear();
            y_value.clear();
            z_value.clear();

            int count = 0;

            for (int k = 0; k < valid_size; k++)
            {
                float cov_alpha = dist_info[k]* i_median_dist;
                if (cov_alpha > 4.f) // original
                {
                    // keypoint is an outlier
                }
                else
                {
                    int ind = index[k];
                    x_value.push_back(depth_joints[i * n_bodyparts + ind].x);
                    y_value.push_back(depth_joints[i * n_bodyparts + ind].y);
                    z_value.push_back(depth_joints[i * n_bodyparts + ind].z);
                    count++;
                }
            }

            if (count > 0)
            {
                middle = (x_value.size()) / 2;

                std::nth_element(x_value.begin(), x_value.begin() + middle, x_value.end());
                std::nth_element(y_value.begin(), y_value.begin() + middle, y_value.end());
                std::nth_element(z_value.begin(), z_value.begin() + middle, z_value.end());

                // ------------------------------------------------------

                Eigen::Vector3d barycenter(x_value[middle], y_value[middle], z_value[middle]);
                zed_tracking::Detection det(world_ref_keypoints,
                            barycenter,
                            currentTime,
                            (double) (barycenter(2)));
                            //(double) (barycenter.norm()));
                    //std::cout << det.getConfidence() << ", ";
                    if (det.getConfidence() > 0.0)
                        detections.push_back(det);
            }
#endif
        }
        //std::cout << std::endl;
        detections_ = detections;

        for (std::list<zed_tracking::PersonTrack*>::iterator it = tracks_.begin(), end = tracks_.end(); it != end;) {
            zed_tracking::PersonTrack* t = *it;
            bool deleted = false;

            // If the track either became old or was never validated before it can be considered fake
            // the track is deleted
            if (((t->getVisibility() == zed_tracking::PersonTrack::NOT_VISIBLE &&
                    (t->getSecFromLastHighConfidenceDetection(currentTime)) >= sec_before_old_)
                    || (!t->isValidated() && t->getSecFromFirstDetection(currentTime) >= sec_before_fake_))) {
                delete t;
                it = tracks_.erase(it);
                deleted = true;
            }// If the track isn't yet validated, but it was updated, with confidence, a certain number
                // of times, the track is valdiated
            else if (!t->isValidated() && t->getUpdatesWithEnoughConfidence() == detections_to_validate_) {
                t->validate();
            }// If the track is validated but still has status NEW, check if a certain amount of time
                // has passed since its first detection and if so set status to NORMAL
            else if (t->getStatus() == zed_tracking::PersonTrack::NEW &&
                    t->getSecFromFirstDetection(currentTime) >= sec_remain_new_) {
                t->setStatus(zed_tracking::PersonTrack::NORMAL);

            }

            // If the track hasn't been deleted because it was too old or was considered fake
            if (!deleted) {
                // If it's NEW and VISIBLE
                if (t->getStatus() == zed_tracking::PersonTrack::NEW && t->getVisibility() == zed_tracking::PersonTrack::VISIBLE) {
                    new_tracks_.push_back(t);
                }
                // If it's NOT_VISIBLE
                if (t->getVisibility() == zed_tracking::PersonTrack::NOT_VISIBLE) {
                    lost_tracks_.push_back(t);
                }
                it++;
            }

        }
    }

    void PersonTracker::updateTracks() {
        createDistanceMatrix();
        createCostMatrix();

        zed_tracking::Munkres munkres;
        cost_matrix_ = munkres.solve(cost_matrix_, false);

        updateDetectedTracks();
        fillUnassociatedDetections();
        updateLostTracks();
        createNewTracks();

    }

    void PersonTracker::createCostMatrix() {
        cost_matrix_ = distance_matrix_.clone();
        for (int i = 0; i < distance_matrix_.rows; i++)
            for (int j = 0; j < distance_matrix_.cols; j++)
                if (distance_matrix_(i, j) > gate_distance_)
                    cost_matrix_(i, j) = 1000000.0;
    }

    void PersonTracker::fillUnassociatedDetections() {
        if (cost_matrix_.cols == 0 && detections_.size() > 0) {
            for (size_t measure = 0; measure < detections_.size(); measure++) {
                unassociated_detections_.push_back(detections_[measure]);
            }
        } else {
            for (int measure = 0; measure < cost_matrix_.cols; measure++) {
                bool associated = false;
                for (int track = 0; track < cost_matrix_.rows; track++) {
                    if (cost_matrix_(track, measure) == 0.0) {
                        if (distance_matrix_(track, measure) > gate_distance_)
                            break;
                        associated = true;
                    }
                }
                if (!associated)
                    unassociated_detections_.push_back(detections_[measure]);
            }
        }
    }

    void PersonTracker::createDistanceMatrix() {
        // Creates a matrix representing the distance between active tracks 
        // and the current time detections
        distance_matrix_ = cv::Mat_<double>(tracks_.size(), detections_.size());

        int track = 0;
        for (std::list<zed_tracking::PersonTrack*>::const_iterator it = tracks_.begin(),
                end = tracks_.end(); it != end; it++) {
            zed_tracking::PersonTrack* t = *it;
            int measure = 0;
            for (std::vector<zed_tracking::Detection>::iterator dit = detections_.begin(), dend = detections_.end();
                    dit != dend; dit++) {
                // Acount for detection confidence
                double detector_likelihood;
                if (detector_likelihood_) {
                    detector_likelihood = dit->getConfidence();
                } else {
                    detector_likelihood = 0;
                }

                // Account for motion likelihood
                double motion_likelihood = t->getMahalanobisDistance(
                        dit->getWorldCentroid()(0),
                        dit->getWorldCentroid()(2),
                        dit->getTime());

                // Compute joint likelihood
                distance_matrix_(track, measure++) = likelihood_weights_[0] * detector_likelihood +
                        likelihood_weights_[1] * motion_likelihood;

                // Clean NaN and infs
                if (std::isnan(distance_matrix_(track, measure - 1)) or
                        (not std::isfinite(distance_matrix_(track, measure - 1))))
                    distance_matrix_(track, measure - 1) = 2 * gate_distance_;
            }
            track++;
        }
        //std::cout << distance_matrix_ << std::endl;
    }

    void PersonTracker::updateDetectedTracks() {
        int track = 0;
        for (std::list<zed_tracking::PersonTrack*>::iterator it = tracks_.begin(), end = tracks_.end();
                it != end; it++) {
            bool updated = false;
            zed_tracking::PersonTrack* t = *it;

            for (int measure = 0; measure < cost_matrix_.cols; measure++) {
                // If there's already a track linked to the detection
                if (cost_matrix_(track, measure) == 0.0 &&
                        distance_matrix_(track, measure) <= gate_distance_) {
                    zed_tracking::Detection& d = detections_[measure];

                    // If the the detection has a minimum confidence in
                    // the current frame or a recent one
                    if ((t->getLowConfidenceConsecutiveFrames() < 10) or
                            (d.getConfidence() > ((min_confidence_ + min_confidence_detections_) / 2))) {
                        bool first_update = false;
                        t->update(d.getWorldCentroid()(0), d.getWorldCentroid()(1),
                                d.getWorldCentroid()(2), d.getHeight(), d.getDistance(),
                                distance_matrix_(track, measure), 100 * d.getConfidence(),
                                min_confidence_, min_confidence_detections_, d.getTime(),
                                d.getJoints(), first_update);

                        t->setVisibility(d.isOccluded() ? zed_tracking::PersonTrack::OCCLUDED : zed_tracking::PersonTrack::VISIBLE);
                        updated = true;
                        break;
                    }
                }
            }

            // Update visibility on not detected tracks
            if (!updated)
                if (t->getVisibility() != zed_tracking::PersonTrack::NOT_VISIBLE)
                    t->setVisibility(zed_tracking::PersonTrack::NOT_VISIBLE);
            track++;
        }
    }

    void PersonTracker::createNewTracks() {
        for (std::list<zed_tracking::Detection>::iterator dit = unassociated_detections_.begin(),
                dend = unassociated_detections_.end(); dit != dend; dit++) {
            createNewTrack(*dit);
        }
    }

    int PersonTracker::createNewTrack(Detection& detection) {
        zed_tracking::PersonTrack* t;

        t = new PersonTrack(
                ++tracks_counter_, world_frame_id_,
                position_variance_, acceleration_variance_,
                period_, velocity_in_motion_term_, detection.getJoints());

        t->init(detection.getWorldCentroid()(0), detection.getWorldCentroid()(1),
                detection.getWorldCentroid()(2), detection.getHeight(),
                detection.getDistance(), detection.getTime(), detection.getJoints());

        bool first_update = true;

        t->update(detection.getWorldCentroid()(0), detection.getWorldCentroid()(1),
                detection.getWorldCentroid()(2), detection.getHeight(),
                detection.getDistance(), 0.0, detection.getConfidence(),
                min_confidence_, min_confidence_detections_, detection.getTime(),
                detection.getJoints(), first_update);

        if (debug_mode_)
            std::cout << "Created Track with ID: " << t->getId() << std::endl;

        tracks_.push_back(t);
        return tracks_counter_;
    }

    /*
        This function will return all PersonTracks from Visibile or Partially occluded people
     */
    //std::vector<std::tuple<int, Eigen::Vector3d, std::vector<Eigen::Vector4d>, std::vector<zed_tracking::Track3DEuro::Visibility>>>

    std::vector<zed_tracking::PeopleSkeletonOutput> PersonTracker::getTrackedPeople() {
        //std::vector<std::tuple<int, Eigen::Vector3d, std::vector<Eigen::Vector4d>, std::vector < zed_tracking::Track3DEuro::Visibility>>> tracked_people;
        std::vector<zed_tracking::PeopleSkeletonOutput> tracked_people;
        // Iterate over tracked people
        for (std::list<zed_tracking::PersonTrack*>::iterator it = tracks_.begin(),
                end = tracks_.end(); it != end; it++) {
            zed_tracking::PersonTrack* t = *it;
            if (t->getVisibility() == zed_tracking::PersonTrack::NOT_VISIBLE) {
                continue;
            }
            if (!t->isValidated()) {
                continue;
            }
            int id = t->getId();
            std::vector<zed_tracking::Track3DEuro*> kps = t->getKeypoints();
            std::vector<sl::float4> all_kps;
            std::vector<zed_tracking::Track3DEuro::Visibility> all_vis;
            Eigen::Vector3d barycenter_eigen = t->getBarycenter();
            std::vector<sl::float3> world_bbox;
            std::vector<int> world_bbox_links = t->getBoundingBoxLinks();
            std::vector<sl::Transform> joint_angles = t->getJointAngles();

            // Iterate over each kp from each tracked person
            
            for (std::vector<zed_tracking::Track3DEuro*>::iterator dit = kps.begin(), dend = kps.end(); dit != dend; dit++) {
                zed_tracking::Track3DEuro* kp_track = *dit;
                double x, y, z, confidence;
                kp_track->getState(x, y, z);
                confidence = kp_track->getLastDetectorConfidence();
                // Create coord + confidence
                sl::float4 kp_coords(x, y, z, confidence);
                //Eigen::Vector4d kp_coords(x, z, y, confidence);
                // Push it back into kp person vector
                zed_tracking::Track3DEuro::Visibility vis = kp_track->getVisibility();
                all_vis.push_back(vis);
                all_kps.push_back(kp_coords);
            }
            

            //all_kps = t->getJointsFitted();

            for (std::vector<zed_tracking::Track3DEuro*>::iterator dit = kps.begin(), dend = kps.end(); dit != dend; dit++) {
                zed_tracking::Track3DEuro *kp_track = *dit;
                zed_tracking::Track3DEuro::Visibility vis = kp_track->getVisibility();
                all_vis.push_back(vis);
            }

            std::vector<Eigen::Vector3d> bbox = t->getBoundingBox();
            //push back world_bbox coords
            for (int i = 0; i < 8; i++) {
                world_bbox.push_back(sl::float3((float) bbox[i](0), (float) bbox[i](1), (float) bbox[i](2)));
                //std::cout << world_bbox[i].x << "," << world_bbox[i].y << "," << world_bbox[i].z << std::endl;
            }

            sl::float3 barycenter(barycenter_eigen(0), barycenter_eigen(1), barycenter_eigen(2));
            sl::float3 speed = getSpeed(id);
            zed_tracking::PeopleSkeletonOutput person;
            person.speed = speed;
            person.id = id;
            person.barycenter = barycenter;
            person.keypoints = all_kps;
            person.keypoint_number = 25;
            person.keypoints_links = {
                    0, 1, 1, 2, 2, 3, 3, 4, 1, 5, 5, 6, 6, 7, 1, 8, 8, 9, 9, 10,
                    10, 11, 11, 22, 11, 24, 8, 12, 12, 13, 13, 14, 14, 19, 14, 21};
            person.world_bbox = world_bbox;
            person.world_bbox_links = world_bbox_links;
            person.joint_angles = joint_angles;
            person.joint_angle_trios = {
                    4,3,2, 3,2,1, 2,1,5, 1,5,6, 5,6,7, 1,0,15, 1,0,16, 0,15,17, 0,16,18, 0,1,8,
                    1,8,9, 1,8,12, 8,9,10, 8,12,13, 9,10,11, 12,13,14, 10,11,22, 10,11,23,
                    10,11,24, 13,14,19, 13,14,20, 13,14,21};
            person.gaze_direction = getGazeDirection(id);
            person.body_orientation = getBodyDirection(id);
            tracked_people.push_back(person);
        }
        return tracked_people;
    }

    sl::float3 PersonTracker::getKP(int person_idx, int body_part_idx) {
        zed_tracking::PersonTrack* t;
        bool found = false;
        for (std::list<zed_tracking::PersonTrack*>::iterator it = tracks_.begin(), end = tracks_.end(); it != end && !found; it++) {
            t = *it;
            if (t->getId() == person_idx)
                found = true;
        }
        if (t->areJointsInitialized() && found) {
            Eigen::Vector3d vec = t->getKP(body_part_idx);
            sl::float3 aux(vec(0), vec(1), vec(2));
            return aux;
        } else {
            sl::float3 pos(NAN, NAN, NAN);
            return pos;
        }
    }

    sl::float3 PersonTracker::getGazeDirection(int person_id) {
        bool found = false;
        zed_tracking::PersonTrack* t;
        for (std::list<zed_tracking::PersonTrack*>::iterator it = tracks_.begin(), end = tracks_.end(); it != end && !found; it++) {
            t = *it;
            int id = t->getId();
            if (id == person_id)
                found = true;
        }
        if (found) {
            Eigen::Vector3d vec = t->getGazeDirection();
            sl::float3 aux(vec(0), vec(1), vec(2));
            return aux;
        }

        sl::float3 aux(NAN, NAN, NAN);
        return aux;
    }

    sl::float3 PersonTracker::getBodyDirection(int person_id) {
        bool found = false;
        zed_tracking::PersonTrack* t;
        for (std::list<zed_tracking::PersonTrack*>::iterator it = tracks_.begin(), end = tracks_.end(); it != end && !found; it++) {
            t = *it;
            int id = t->getId();
            if (id == person_id)
                found = true;
        }
        if (found) {
            Eigen::Vector3d vec = t->getBodyDirection();
            sl::float3 aux(vec(0), vec(1), vec(2));
            return aux;
        }

        sl::float3 aux(NAN, NAN, NAN);
        return aux;
    }

    sl::float3 PersonTracker::getSpeed(int person_id) {
        bool found = false;
        zed_tracking::PersonTrack* t;
        for (std::list<zed_tracking::PersonTrack*>::iterator it = tracks_.begin(), end = tracks_.end(); it != end && !found; it++) {
            t = *it;
            int id = t->getId();
            if (id == person_id)
                found = true;
        }
        if (found) {
            sl::float3 vec = t->getSpeed();
            return vec;
        }

        sl::float3 vec(NAN, NAN);
        return vec;
    }

} /*namespace zed_tracking*/
