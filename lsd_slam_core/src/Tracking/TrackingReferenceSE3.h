
/*
 * July. 4, 2016, David Z
 * 
 * tracking reference to support SE3
 *
 * */

#ifndef TRACKING_REFERENCE_SE3
#define TRACKING_REFERENCE_SE3

#include "util/settings.h"
#include "util/EigenCoreInclude.h"
#include "boost/thread/mutex.hpp"
#include <boost/thread/shared_mutex.hpp>


namespace lsd_slam
{

// class Frame;
class FrameSE3; 
class DepthMapPixelHypothesis;
// class KeyFrameGraph;
class KeyFrameGraphSE3;

/**
 * Point cloud used to track frame poses.
 * 
 * Basically this stores a point cloud generated from known frames. It is used to
 * track a new frame by finding a projection of the point cloud which makes it
 * look as much like the new frame as possible.
 * 
 * It is intended to use more than one old frame as source for the point cloud.
 * Also other data like Kinect depth data could be imported.
 * 
 * ATTENTION: as the level zero point cloud is not used for tracking, it is not
 * fully calculated. Only the weights are valid on this level!
 */
class TrackingReferenceSE3
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/** Creates an empty TrackingReference with optional preallocation per level. */
	TrackingReferenceSE3();
	~TrackingReferenceSE3();
	void importFrame(FrameSE3* source);

	FrameSE3* keyframe;
	boost::shared_lock<boost::shared_mutex> keyframeLock;
	int frameID;

	void makePointCloud(int level);
	void clearAll();
	void invalidate();
	Eigen::Vector3f* posData[PYRAMID_LEVELS];	// (x,y,z)
	Eigen::Vector2f* gradData[PYRAMID_LEVELS];	// (dx, dy)
	Eigen::Vector2f* colorAndVarData[PYRAMID_LEVELS];	// (I, Var)
	int* pointPosInXYGrid[PYRAMID_LEVELS];	// x + y*width
	int numData[PYRAMID_LEVELS];

private:
	int wh_allocated;
	boost::mutex accessMutex;
	void releaseAll();
};
}

#endif
