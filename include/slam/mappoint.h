#pragma once
#ifndef SLAM_MAPPOINT_H
#define SLAM_MAPPOINT_H


#include "slam/common_include.h"



namespace slam
{
    
struct Frame;
struct Feature;


 // namespace slam

// Mappoints represents a 3D point in the world (landmark)

// A mappoint is created by trangulating matching feature points from stereo view.

struct MapPoint
{
public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    using Ptr = std::shared_ptr<MapPoint>;

    // unique id for the mappoint

    unsigned long id_ = 0;

    // whether this point is considerd an outlier
    bool is_outlier_ = false;

    // 3D position in the world coordinates
    Vec3 pose_ = Vec3::Zero();

    // Mutex

    std::mutex data_mutex_ ;

    // number of times this point has been observed
    int observed_times_ = 0;

    // features that observe this mappoint

    std::list<std::weak_ptr<Feature>> observations_;


public:

    // constructor
    MapPoint() = default;

    // constructor with id and initial position

    MapPoint(long id, Vec3 position);

    // get current position

    Vec3 Pose(){
        std::unique_lock<std::mutex> lock(data_mutex_);
        pos_ = pos;
    }

    // add an observation

    void AddObservation(std::shared_ptr<Feature> feature) {
        std::unique_lock<std::mutex> lock(data_mutex_);
        observations_.push_back(feature);
        observed_times_++;
    }

        /// Remove an observation
    void RemoveObservation(std::shared_ptr<Feature> feat);

    /// Get all observations (thread-safe)
    std::list<std::weak_ptr<Feature>> GetObs() {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return observations_;
    }

    /// Factory function to create a new MapPoint with a unique ID
    static MapPoint::Ptr CreateNewMappoint();

};



}

#endif 