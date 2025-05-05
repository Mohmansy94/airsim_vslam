
#pragma once
#ifndef MAP_H
#define MAP_H

#include "slam/common_include.h"
#include "slam/frame.h"
#include "slam/mappoint.h"

namespace slam {
class Map {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        using Ptr = std::shared_ptr<Map>;
        using LandmarksType = std::unordered_map<unsigned long, MapPoint::Ptr>;
        using KeyframesType = std::unordered_map<unsigned long, Frame::Ptr>;
    
        Map() = default;
    
        /// Insert a new keyframe
        void InsertKeyFrame(Frame::Ptr frame);
    
        /// Insert a new map point
        void InsertMapPoint(MapPoint::Ptr map_point);
    
        /// Get all map points (thread-safe)
        LandmarksType GetAllMapPoints() {
            std::unique_lock<std::mutex> lock(data_mutex_);
            return landmarks_;
        }
    
        /// Get all keyframes (thread-safe)
        KeyframesType GetAllKeyFrames() {
            std::unique_lock<std::mutex> lock(data_mutex_);
            return keyframes_;
        }
    
        /// Get active map points (thread-safe)
        LandmarksType GetActiveMapPoints() {
            std::unique_lock<std::mutex> lock(data_mutex_);
            return active_landmarks_;
        }
    
        /// Get active keyframes (thread-safe)
        KeyframesType GetActiveKeyFrames() {
            std::unique_lock<std::mutex> lock(data_mutex_);
            return active_keyframes_;
        }
    
        /// Remove map points with no observations
        void CleanMap();
    
    private:
        /// Remove the oldest keyframe if number exceeds the limit
        void RemoveOldKeyframe();
    
        /// Mutex for thread safety
        std::mutex data_mutex_;
    
        /// All and active map points
        LandmarksType landmarks_;
        LandmarksType active_landmarks_;
    
        /// All and active keyframes
        KeyframesType keyframes_;
        KeyframesType active_keyframes_;
    
        /// Most recent frame
        Frame::Ptr current_frame_ = nullptr;
    
        /// Max number of active keyframes
        int num_active_keyframes_ = 7;
    };
    
    }  // namespace slam
    
#endif  // MAP_H