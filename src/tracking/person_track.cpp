#include "tracking/person_track.h"

namespace tracker {

    int PersonTrack::count = 0;

    PersonTrack::PersonTrack(int id,
            std::string frame_id, double position_variance,
            double acceleration_variance, double period,
            bool velocity_in_motion_term,
            const std::vector<Eigen::Vector4d>& joints) :
    Track(id, frame_id, position_variance, acceleration_variance, period,
    velocity_in_motion_term), all_joint_tracks_initialized_(false) {
        joint_tracks_.resize(tracker::PoseJoints::SIZE);
        for (size_t i = 0; i < tracker::PoseJoints::SIZE; ++i) {
            if (i == 3 or i == 6 or i == 10 or i == 13) {
                // Knees + Elbows
                joint_tracks_[i] = new Track3DEuro(id,
                        frame_id,
                        period,
                        0.6, 1, 1); // 5, 4, 1 //0.8, 10, 1
            } else {
                if (i == 4 or i == 7 or i == 11 or i == 14 or i == 19 or i == 20 or i == 21 or i == 22 or i == 23 or i == 24) {
                    // Hands + feet
                    joint_tracks_[i] = new Track3DEuro(id,
                            frame_id,
                            period,
                            0.7, 1, 1); // 5, 4, 1 //0.8, 10, 1
                } else {
                    joint_tracks_[i] = new Track3DEuro(id,
                            frame_id,
                            period,
                            0.4, 1, 1); //1, 0, 1 //0.8, 5, 1
                }
            }
        }

        bbox_vertices_.resize(8);
        for (size_t i = 0; i < 8; i++)
            bbox_vertices_[i] = new OneEuroFilter3D(1. / period, 0.05, 1, 1);

        /*
        body = new slBody25KinChain();
        for (unsigned i = 0; i < NUM_OF_DIMENSIONS; i++){
            if (i < 25){
                tracker::float4 kp = {NAN, NAN, NAN, NAN};
                fitte_keypoints_.push_back(kp);
            }
            last_optim_state_[i] = 0
        }
        */

        debug_count_ = -1;
    }

    PersonTrack::~PersonTrack() {
        PersonTrack::count++;
        //delete body;
    }

    bool
    PersonTrack::anyNaNs(const std::vector<Eigen::Vector4d>& joints) {
        for (size_t i = 0; i < tracker::PoseJoints::SIZE; ++i) {
            if (std::isnan(joints[i](0)) or std::isnan(joints[i](1)) or std::isnan(joints[i](2)))
                return true;
        }
        return false;
    }

    void
    PersonTrack::init(double x, double y, double z, double height, double distance,
            struct timeval detection_time,
            const std::vector<Eigen::Vector4d>& joints) {
        //double time_in_sec = ((detection_time.tv_sec*(double)1000000) + (detection_time.tv_usec))/1000000;
        Track::init(x, z, y, height, distance, detection_time);
        bool any_nan = anyNaNs(joints);
        any_nan ? all_joint_tracks_initialized_ = false : all_joint_tracks_initialized_ = true;

        tracker::float3 coord_mins = {NAN, NAN, NAN};
        tracker::float3 coord_maxs = {NAN, NAN, NAN};
        Eigen::Vector3d barycenter_eigen = getBarycenter();

        double time_in_sec = ((detection_time.tv_sec * (double) 1000000) + (detection_time.tv_usec)) / 1000000;

        for (int i = 0, end = joint_tracks_.size(); i != end and not any_nan; ++i) {
            const Eigen::Vector4d& bj = joints[i];
            joint_tracks_[i]->init(bj(0), bj(1), bj(2), 10,
                    Eigen::Vector3d(bj(0), bj(1), bj(2)).norm(),
                    detection_time);

        }
        raw_joints_tmp_ = joints;

    }

    bool
    PersonTrack::areJointsInitialized() {
        return all_joint_tracks_initialized_;
    }

    void
    PersonTrack::update(
            double x,
            double y,
            double z,
            double height,
            double distance,
            double data_assocation_score,
            double confidence,
            double min_confidence,
            double min_confidence_detections,
            struct timeval detection_time,
            const std::vector<Eigen::Vector4d>& joints,
            bool first_update) {

        Track::update(x, z, y, height, distance, data_assocation_score,
                confidence, min_confidence, min_confidence_detections,
                detection_time, first_update);

        if (all_joint_tracks_initialized_) {
            for (int i = 0, end = tracker::PoseJoints::SIZE; i != end; ++i) {
                const Eigen::Vector4d& bj = joints[i];

                joint_tracks_[i]->update(bj(0), bj(1), bj(2), height,
                        Eigen::Vector3d(bj(0), bj(1), bj(2)).norm(), 100 * bj(3),
                        100 * bj(3) - 1, 100 * bj(3) + 1, 0,
                        detection_time, first_update
                        );

            }
        } else {
            bool any_nan = anyNaNs(joints);
            if (not any_nan) {
                all_joint_tracks_initialized_ = true;
                for (int i = 0, end = joint_tracks_.size(); i != end; ++i) {
                    const Eigen::Vector4d& bj = joints[i];
                    joint_tracks_[i]->init(bj(0), bj(1), bj(2), 10,
                            Eigen::Vector3d(bj(0), bj(1), bj(2)).norm(),
                            detection_time);

                    bool first_update = true;
                    joint_tracks_[i]->update(
                            bj(0), bj(1), bj(2), 10,
                            Eigen::Vector3d(bj(0), bj(1), bj(2)).norm(), 100 * bj(3),
                            100 * bj(3) - 1, 100 * bj(3) + 1, 0,
                            detection_time, first_update);

                }
            } else {
                all_joint_tracks_initialized_ = true;
                for (int i = 0, end = joint_tracks_.size(); i != end; ++i) {
                    if (isValid(joints[i])) {
                        const Eigen::Vector4d& bj = joints[i];
                        joint_tracks_[i]->init(bj(0), bj(1), bj(2), 10,
                                Eigen::Vector3d(bj(0), bj(1), bj(2)).norm(),
                                detection_time);

                        bool first_update = true;
                        joint_tracks_[i]->update(
                                bj(0), bj(1), bj(2), 10,
                                Eigen::Vector3d(bj(0), bj(1), bj(2)).norm(), 100 * bj(3),
                                100 * bj(3) - 1, 100 * bj(3) + 1, 0,
                                detection_time, first_update);

                    } else {
                        const Eigen::Vector4d bj(0, 0, 0, 0);
                        joint_tracks_[i]->init(bj(0), bj(1), bj(2), 10,
                                Eigen::Vector3d(bj(0), bj(1), bj(2)).norm(),
                                detection_time);
                        bool first_update = true;
                        joint_tracks_[i]->update(
                                NAN, NAN, NAN, 10,
                                NAN, 0,
                                0 - 1, 0 + 1, 0,
                                detection_time, first_update);
                    }
                }
            }
        }
        double time_in_sec = ((detection_time.tv_sec * (double) 1000000) + (detection_time.tv_usec)) / 1000000;

        /*
        // Skeleton Fitting
        fitToSkeleton();
        */

        // Calculate 3D bounding box
        //std::cout << "coords_mins: " << coord_mins.x << "," << coord_mins.y << "," << coord_mins.z << std::endl;
        //std::cout << "coords_maxs: " << coord_maxs.x << "," << coord_maxs.y << "," << coord_maxs.z << std::endl;
        tracker::float3 coord_mins = {NAN, NAN, NAN};
        tracker::float3 coord_maxs = {NAN, NAN, NAN};
        Eigen::Vector3d barycenter_eigen = getBarycenter();
        double sx, sy, sz, c;

        for (unsigned i = 0; i < 25; i++) {
            joint_tracks_[i]->getState(sx, sy, sz);
            c = joint_tracks_[i]->getLastDetectorConfidence();
            // Compute 3D bbox
            float distance_gravity_center = sqrt(pow(sx - barycenter_eigen(0), 2) +
                    pow(sy - barycenter_eigen(1), 2) +
                    pow(sz - barycenter_eigen(2), 2));

            float distance_barycenter_floor_plane = sqrt(pow(sx - barycenter_eigen(0), 2) +
                    pow(sz - barycenter_eigen(2), 2));

            float sum = sx + sy + sz;

            if (c > 0 && distance_gravity_center < 1.4 &&
                    distance_barycenter_floor_plane < 0.8 &&
                    sum != 0) {
                coord_mins.x = std::isfinite(coord_mins.x) ? std::min(coord_mins.x, (float) sx) : sx;
                coord_mins.y = std::isfinite(coord_mins.y) ? std::min(coord_mins.y, (float) sy) : sy;
                coord_mins.c = std::isfinite(coord_mins.c) ? std::min(coord_mins.c, (float) sz) : sz;

                coord_maxs.x = std::isfinite(coord_maxs.x) ? std::max(coord_maxs.x, (float) sx) : sx;
                coord_maxs.y = std::isfinite(coord_maxs.y) ? std::max(coord_maxs.y, (float) sy) : sy;
                coord_maxs.c = std::isfinite(coord_maxs.c) ? std::max(coord_maxs.c, (float) sz) : sz;
            }
        }

        if (std::isfinite(coord_mins.x) && std::isfinite(coord_mins.y) &&
                std::isfinite(coord_mins.c) && std::isfinite(coord_maxs.x) &&
                std::isfinite(coord_maxs.y) && std::isfinite(coord_maxs.c)) {
            bbox_vertices_[0]->update((double) coord_mins.x, (double) coord_maxs.y, (double) coord_maxs.c, time_in_sec, false);
            bbox_vertices_[1]->update((double) coord_maxs.x, (double) coord_maxs.y, (double) coord_maxs.c, time_in_sec, false);
            bbox_vertices_[2]->update((double) coord_mins.x, (double) coord_mins.y, (double) coord_maxs.c, time_in_sec, false);
            bbox_vertices_[3]->update((double) coord_maxs.x, (double) coord_mins.y, (double) coord_maxs.c, time_in_sec, false);

            bbox_vertices_[4]->update((double) coord_mins.x, (double) coord_maxs.y, (double) coord_mins.c, time_in_sec, false);
            bbox_vertices_[5]->update((double) coord_maxs.x, (double) coord_maxs.y, (double) coord_mins.c, time_in_sec, false);
            bbox_vertices_[6]->update((double) coord_mins.x, (double) coord_mins.y, (double) coord_mins.c, time_in_sec, false);
            bbox_vertices_[7]->update((double) coord_maxs.x, (double) coord_mins.y, (double) coord_mins.c, time_in_sec, false);
        }
        raw_joints_tmp_ = joints;
        debug_count_++;
    }

    bool
    PersonTrack::isValid(const Eigen::Vector4d& joint) {
        return std::isfinite(joint(0)) and std::isfinite(joint(1)) and std::isfinite(joint(2));
    }

    bool
    PersonTrack::isValid(const std::vector<tracker::float4>& joint)
    {
        for (unsigned i  = 0; i < joint.size(); i++)
            if ( !std::isfinite(joint[i].x) or !std::isfinite(joint[i].y) or !std::isfinite(joint[i].z) )
                return false;
        return true;
    }

    //Return all VISIBLE or PARTIALLY OCCLUDED tracks

    std::vector<tracker::Track3DEuro*>
    PersonTrack::getKeypoints() {
        return joint_tracks_;
    }

    Eigen::Vector3d
    PersonTrack::getBarycenter() {
        double x, y;
        filter_->getState(x, y);
        Eigen::Vector3d barycenter(x, z_, y);
        return barycenter;
    }

    Eigen::Vector3d
    PersonTrack::getKP(int body_part_idx) {
        std::vector<tracker::Track3DEuro*>::iterator it = joint_tracks_.begin();
        std::advance(it, body_part_idx);
        tracker::Track3DEuro* t = *it;
        double x, y, z;
        t->getState(x, y, z);
        Eigen::Vector3d pos(x, y, z);
        return pos;
    }

    Eigen::Vector3d
    PersonTrack::getGazeDirection() {
        std::vector<tracker::Track3DEuro*>::iterator it = joint_tracks_.begin();
        std::advance(it, 1); // Getting to the neck point (NOT using the nose point)
        tracker::Track3DEuro* t = *it;
        if (t->getVisibility() == tracker::Track3DEuro::NOT_VISIBLE)
            return Eigen::Vector3d(NAN, NAN, NAN);
        double x, y, z;
        t->getState(x, y, z);
        Eigen::Vector3d pos_nose(x, y, z); // May have the nose or neck coordinates, need to test
        std::advance(it, 13); // Getting to the REYE iterator
        t = *it;
        if (t->getVisibility() == tracker::Track3DEuro::NOT_VISIBLE)
            return Eigen::Vector3d(NAN, NAN, NAN);
        t->getState(x, y, z);
        Eigen::Vector3d pos_reye(x, y, z);
        std::advance(it, 1); // Getting to the LEYE iterator
        t = *it;
        if (t->getVisibility() == tracker::Track3DEuro::NOT_VISIBLE)
            return Eigen::Vector3d(NAN, NAN, NAN);
        t->getState(x, y, z);
        Eigen::Vector3d pos_leye(x, y, z);

        // Having the three points, A, B and C, we calculate the normal to the plane defined by them
        // To do so we use the cross product function of Eigen to multiply (B-A)x(C-A)
        pos_reye = pos_reye - pos_nose; //(B-A)
        pos_leye = pos_leye - pos_nose; //(C-A)
        pos_nose = pos_leye.cross(pos_reye);
        //pos_nose now contains the normal vector to the plane, pointing in the direction of the gaze
        Eigen::Vector3d normalized_pos_nose = pos_nose.normalized();
        return normalized_pos_nose;
    }

    Eigen::Vector3d
    PersonTrack::getBodyDirection() {
        std::vector<tracker::Track3DEuro*>::iterator it = joint_tracks_.begin();
        std::vector<Eigen::Vector3d> points;
        bool rs = true, ls = true, rh = true, lh = true;
        // Check RIGHT SHOULDER
        std::advance(it, 2);
        tracker::Track3DEuro* t = *it;
        int not_visible_count = 0;
        if (t->getVisibility() == tracker::Track3DEuro::NOT_VISIBLE) {
            not_visible_count++;
            rs = false;
        }
        double x, y, z;
        t->getState(x, y, z);
        points.push_back(Eigen::Vector3d(x, y, z));
        //Eigen::Vector3d pos_right_shoulder(x, y, z);

        // Check LEFT SHOULDER
        std::advance(it, 3);
        t = *it;
        if (t->getVisibility() == tracker::Track3DEuro::NOT_VISIBLE) {
            if (not_visible_count > 0) {
                return Eigen::Vector3d(NAN, NAN, NAN);
            } else {
                not_visible_count++;
                ls = false;
            }
        }
        t->getState(x, y, z);
        points.push_back(Eigen::Vector3d(x, y, z));
        //Eigen::Vector3d pos_left_shoulder(x, y, z);

        // Check RIGHT HIP
        std::advance(it, 4); // Getting to the LEYE iterator
        t = *it;
        if (t->getVisibility() == tracker::Track3DEuro::NOT_VISIBLE) {
            if (not_visible_count > 0) {
                return Eigen::Vector3d(NAN, NAN, NAN);
            } else {
                not_visible_count++;
                rh = false;
            }
        }
        t->getState(x, y, z);
        points.push_back(Eigen::Vector3d(x, y, z));
        //Eigen::Vector3d pos_leye(x, y, z);

        // Check LEFT HIP
        std::advance(it, 3); // Getting to the LEYE iterator
        t = *it;
        if (t->getVisibility() == tracker::Track3DEuro::NOT_VISIBLE) {
            if (not_visible_count > 0) {
                return Eigen::Vector3d(NAN, NAN, NAN);
            } else {
                not_visible_count++;
                lh = false;
            }
        }
        t->getState(x, y, z);
        points.push_back(Eigen::Vector3d(x, y, z));
        //Eigen::Vector3d pos_leye(x, y, z);

        Eigen::Vector3d D(0, 0, 0); //(B-A)
        Eigen::Vector3d E(0, 0, 0); //(B-A)
        // Having the three points, A, B and C, we calculate the normal to the plane defined by them
        // To do so we use the cross product function of Eigen to multiply (B-A)x(C-A)
        if (rs && ls && rh && lh) // all
        {
            D = points[2] - points[0]; //(B-A)
            E = points[1] - points[0]; //(B-A)
        } else {
            if (ls && rh && lh) // not rs
            {
                D = points[1] - points[0]; //(B-A)
                E = points[2] - points[0]; //(B-A)
            } else {
                if (rs && rh && lh) // not ls
                {
                    D = points[1] - points[0]; //(B-A)
                    E = points[2] - points[0]; //(B-A)
                } else {
                    if (rs && ls && lh) // not rh
                    {
                        D = points[2] - points[0]; //(B-A)
                        E = points[1] - points[0]; //(B-A)
                    } else { // not lh
                        D = points[2] - points[0]; //(B-A)
                        E = points[1] - points[0]; //(B-A)
                    }
                }
            }
        }
        D = E.cross(D);
        //pos_nose now contains the normal vector to the plane, pointing in the direction of the gaze
        Eigen::Vector3d normalized_body_direction = D.normalized();
        return normalized_body_direction;
    }

    std::vector<Eigen::Vector3d>
    PersonTrack::getBoundingBox() {
        std::vector<Eigen::Vector3d> bbox;
        double x, y, z;
        for (unsigned i = 0; i < 8; i++) {
            bbox_vertices_[i]->getState(x, y, z);
            Eigen::Vector3d aux((float) x, (float) y, (float) z);
            bbox.push_back(aux);
        }
        return bbox;
    }

    std::vector<int>
    PersonTrack::getBoundingBoxLinks() {
        std::vector<int> boxLinks = {0, 1, 0, 2, 0, 4, 6, 2, 6, 7, 6, 4, 3, 2, 3, 7, 3, 1, 5, 7, 5, 1, 4, 5};
        return boxLinks;
    }

    std::vector<tracker::float4>
    PersonTrack::getJointsPosition() {
        std::vector<tracker::float4> all_kps;
        for (std::vector<tracker::Track3DEuro*>::iterator it = joint_tracks_.begin(), end = joint_tracks_.end(); it != end; it++) {
            tracker::Track3DEuro* kp = *it;
            double x, y, z, confidence;
            kp->getState(x, y, z);
            confidence = kp->getLastDetectorConfidence();
            tracker::float4 kp_coords = {x, y, z, confidence};
            all_kps.push_back(kp_coords);
        }
        return all_kps;
    }

    /*  getJointAngles()

        Compute the joint angles -> rotation matrix from limb A to limb B,
        joined by the joint in question.
     */
    std::vector<Eigen::Affine3f>
    PersonTrack::getJointAngles() {
        /*
        // 22 trios
        std::vector<int> joint_trios = {2, 3, 4, //RWRIST, RELBOW, RSHOULDER
            1, 2, 3, //RELBOW, RSHOULDER, NECK
            2, 1, 5, //RSHOULDER, NECK, LSHOULDER
            1, 5, 6, //NECK, LSHOULDER, LELBOW
            5, 6, 7, //LSHOULDER, LELBOW, LWRIST
            1, 0, 15, //NECK, NOSE, REYE
            1, 0, 16, //NECK, NOSE, LEYE
            0, 15, 17, //NOSE, REYE, REAR
            0, 16, 18, //NOSE, LEYE, LEAR
            0, 1, 8, //NOSE, NECK, MIDHIP
            1, 8, 9, //NECK, MIDHIP, RHIP
            1, 8, 12, //NECK, MIDHIP, LHIP
            8, 9, 10, //MIDHIP, RHIP, RKNEE
            8, 12, 13, //MIDHIP, LHIP, LKNEE
            9, 10, 11, //RHIP, RKNEE, RANKLE
            12, 13, 14, //LHIP, LKNEE, LANKLE
            10, 11, 22, //RKNEE, RANKLE, RBIGTOE
            10, 11, 23, //RKNEE, RANKLE, RSMALLTOE
            10, 11, 24, //RKNEE, RANKLE, RHEEL
            13, 14, 19, //LKNEE, LANKLE, LBIGTOE
            13, 14, 20, //LKNEE, LANKLE, LSMALLTOE
            13, 14, 21 //LKNEE, LANKLE, LHEEL
        };
        std::vector<tracker::float4> joint_positions = getJointsPosition();
        std::vector<Eigen::Affine3f> joint_angles;
        for (unsigned i = 0; i < joint_trios.size() / 3; i++) {
            // Compute the two limbs A and B
            tracker::float4 limbA = joint_positions[3 * i + 1] - joint_positions[3 * i];
            tracker::float4 limbB = joint_positions[3 * i + 2] - joint_positions[3 * i + 1];

            // Unormalized vectors, passing to Eigen
            Eigen::Vector3d vectorA(limbA.x, limbA.y, limbA.z);
            Eigen::Vector3d vectorB(limbB.x, limbB.y, limbB.z);

            // Normalize vectors to simplify trigonometric computations
            vectorA.normalize();
            vectorB.normalize();

            // Calculate the vectors cross product norm and dot product
            auto crossProd = vectorA.cross(vectorB);
            float crossNorm = (float) crossProd.norm();
            float dotProd = (float) vectorA.dot(vectorB);

            // Calculate the rotation matrix in an auxiliary base (proj(B,A), rej(B,A), cross(B,A))
            Eigen::Matrix3f rotation = Eigen::Matrix3f::Zero();
            rotation(0, 0) = dotProd;
            rotation(0, 1) = -crossNorm;
            rotation(1, 0) = crossNorm;
            rotation(1, 1) = dotProd;
            rotation(2, 2) = 1;

            // Compute auxiliary base vectors for anti transform -> base = (u, v, w)
            Eigen::Vector3d u = vectorA;
            Eigen::Vector3d v = vectorB - dotProd*vectorA;
            v.normalize();
            Eigen::Vector3d w = crossProd;

            // Compute the transform from the auxiliary base onto the initial base
            float dat[] = {u(0), v(0), w(0),
                u(1), v(1), w(1),
                u(2), v(2), w(2)};
            Eigen::Matrix3f fInv(dat);

            // Reproject transform onto initial base
            Eigen::Matrix3f transf = fInv * rotation * fInv.inverse();

            // Copy results to sl::Transform
            sl::Transform joint_transform;
            float data[] = {transf(0, 0), transf(0, 1), transf(0, 2),
                transf(1, 0), transf(1, 1), transf(1, 2),
                transf(2, 0), transf(2, 1), transf(2, 2)};
            sl::Matrix3f mat(data);
            sl::Rotation rot(mat);
            joint_transform.setRotation(rot);

            joint_angles.push_back(joint_transform);
        }

        return joint_angles;
        */
        std::vector<Eigen::Affine3f> return_angles;
        return return_angles;
    }

    /*
    std::vector<tracker::float4>
    PersonTrack::getJointsFitted()
    {
        return fitted_keypoints_;
    }
    */

    /*
    void
    PersonTrack::fitToSkeleton()
    {
        // Create optimizable values pointer
        float positions[NUM_OF_PARTICLES * NUM_OF_DIMENSIONS];

        // Create velocities pointer
        float velocities[NUM_OF_PARTICLES * NUM_OF_DIMENSIONS];

        // Create particle's best positions pointer
        float pBests[NUM_OF_PARTICLES * NUM_OF_DIMENSIONS];

        // Create global best position pointer
        float gBest[NUM_OF_DIMENSIONS];

        bool validPoints = isValid(fitted_keypoints_);

        for (unsigned i = 0; i < NUM_OF_DIMENSIONS * NUM_OF_PARTICLES; i++)
        {
            // If it's the first particle, initialize with last known fitted position
            if ( (i < NUM_OF_DIMENSIONS) and (validPoints) )
            {
                positions[i] = last_optim_state_[i];
            }
            else // Initialize other particles "randomly"
            {
               positions[i] = getRandom(START_RANGE_MIN[i % 35], START_RANGE_MAX[i % 35]);
            }

            pBests[i] = positions[i];
            velocities[i] = getRandomClamped();

            // Initialize best position as last known fitted position
            if (i < NUM_OF_DIMENSIONS)
                gBest[i] = pBests[i];
        }

        // Optimize variable body parameters with Particle Swarm Optimization process
        std::vector<tracker::float4> kps = getJointsPosition();
        float keypoints[3*25];
        for (unsigned i = 0; i < 25; i++)
        {
            keypoints[3*i] = kps[i].x;
            keypoints[3*i+1] = kps[i].y;
            keypoints[3*i+2] = kps[i].z;
            //std::cout << kps[i].x << ", " << kps[i].y << ", " << kps[i].z;
        }
        //std::cout << std::endl << std::endl;
        pso(positions, velocities, pBests, gBest, body, keypoints);

        // Save best state
        for (unsigned i = 0; i < NUM_OF_DIMENSIONS; i++)
        {
            last_optim_state_[i] = gBest[i];
            //std::cout << gBest[i] << ", ";
        }
        //std::cout << std::endl << std::endl;

        // Translate axis, angles, position and scale into standard position
        body->resetDefaultPose();

        // Individually set joint rotations, and apply forward kinematics
        // to retrieve the global positions
        Eigen::Vector3f j8_joint_angle(gBest[0], gBest[1], gBest[2]);
        body->jointSkeleton[8]->setThisLocalAxisAngleParameter(j8_joint_angle);

        Eigen::Vector3f j2_joint_angle(gBest[3], gBest[4], gBest[5]);
        body->jointSkeleton[2]->setThisLocalAxisAngleParameter(j2_joint_angle);

        Eigen::Vector3f j3_joint_angle(0, 0, gBest[6]);
        body->jointSkeleton[3]->setThisLocalAxisAngleParameter(j3_joint_angle);

        Eigen::Vector3f j5_joint_angle(gBest[7], gBest[8], gBest[9]);
        body->jointSkeleton[5]->setThisLocalAxisAngleParameter(j5_joint_angle);

        Eigen::Vector3f j6_joint_angle(0, 0, gBest[10]);
        body->jointSkeleton[6]->setThisLocalAxisAngleParameter(j6_joint_angle);

        Eigen::Vector3f j9_joint_angle(gBest[11], gBest[12], gBest[13]);
        body->jointSkeleton[9]->setThisLocalAxisAngleParameter(j9_joint_angle);

        Eigen::Vector3f j10_joint_angle(gBest[14], 0, 0);
        body->jointSkeleton[10]->setThisLocalAxisAngleParameter(j10_joint_angle);

        Eigen::Vector3f j12_joint_angle(gBest[15], gBest[16], gBest[17]);
        body->jointSkeleton[12]->setThisLocalAxisAngleParameter(j12_joint_angle);

        Eigen::Vector3f j13_joint_angle(gBest[18], 0, 0);
        body->jointSkeleton[13]->setThisLocalAxisAngleParameter(j13_joint_angle);

        Eigen::Vector3f j11_joint_angle(gBest[19], gBest[20], gBest[21]);
        body->jointSkeleton[11]->setThisLocalAxisAngleParameter(j11_joint_angle);

        Eigen::Vector3f j14_joint_angle(gBest[22], gBest[23], gBest[24]);
        body->jointSkeleton[8]->setThisLocalAxisAngleParameter(j8_joint_angle);

        Eigen::Vector3f j1_up_joint_angle(gBest[25], gBest[26], gBest[27]);
        body->jointSkeleton[26]->setThisLocalAxisAngleParameter(j1_up_joint_angle);

        body->setXYZScale(gBest[34], gBest[35], gBest[36]);

        //Eigen::Vector3f orientation(gBest[28], gBest[29], gBest[30]);
        //Eigen::Vector3f translation(gBest[31], gBest[32], gBest[33]);
        Eigen::Matrix<float,6,1> orientation;
        orientation << gBest[28], gBest[29], gBest[30], gBest[31], gBest[32], gBest[33];
        body->setRootTransform(orientation);

        //for (unsigned i = 0; i < 35; i++)
        //    printf("%f, ", gBest[i]);
        //printf("\n");

        // Forward kinematics
        slKinNode::updateForward(body->root);

        // Retrieve keypoints positions
        std::vector<double> all_confs;
        for (std::vector<tracker::Track3DEuro*>::iterator it = joint_tracks_.begin(), end = joint_tracks_.end(); it != end; it++)
        {
            tracker::Track3DEuro* kp = *it;
            double confidence;
            confidence = kp->getLastDetectorConfidence();
            all_confs.push_back(confidence);
        }
        for (unsigned i = 0; i < 25; i++)
        {
            Eigen::Matrix4f transform = body->jointSkeleton[i]->getGlobalJointPose();
            sl::float4 fitted_kp(transform(0,3), transform(1,3), transform(2,3), all_confs[i]);
            //printf("%f, %f, %f, %f\n", fitted_kp.x, fitted_kp.y , fitted_kp.z , fitted_kp.w);
            fitted_keypoints_[i] = fitted_kp;
        }
        //printf("\n");

    }
    */

} /*namespace tracker*/
