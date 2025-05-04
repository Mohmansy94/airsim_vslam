# pragma

#ifndef SLAM_FRAME_H
#define SLAME_FRAME_H

#include"slam/camera.h"

#include"slam/common_include.h"

namespace slam
{
 
struct MapPoint;
struct Feature;


// Each frame stors stereo images, extract features, and a pose. 
// some frames are marked as keyframes
struct Frame {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    using Ptr = std::shared_ptr<Frame>;


}

    


} // namespace slam





#endif