#ifndef TRACKABLE_KEYFRAME_SEARCH_SE3_H
#define TRACKABLE_KEYFRAME_SEARCH_SE3_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <Eigen/StdVector>
#include "util/SophusUtil.h"

#ifdef HAVE_FABMAP
	#include "GlobalMapping/FabMap.h"
#endif

#include "util/settings.h"



namespace lsd_slam
{


// class KeyFrameGraph;
class KeyFrameGraphSE3; 
class SE3Tracker;
// class Frame;
class FrameSE3; 

struct TrackableKFStructSE3
{
	FrameSE3* ref;
	SE3 refToFrame;
	float dist;
	float angle;
};

/**
 * Given a KeyFrame, tries to find other KeyFrames from a KeyFrameGraph which
 * can be tracked from this frame (in order to insert new constraints into
 * the graph).
 */
class TrackableKeyFrameSearchSE3
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/** Constructor. */
	TrackableKeyFrameSearchSE3(KeyFrameGraphSE3* graph, int w, int h, Eigen::Matrix3f K);
	~TrackableKeyFrameSearchSE3();
	
	/**
	 * Finds candidates for trackable frames.
	 * Returns the most likely candidates first.
	 */
	std::unordered_set<FrameSE3*, std::hash<FrameSE3*>, std::equal_to<FrameSE3*>, Eigen::aligned_allocator< FrameSE3* > > findCandidates(FrameSE3* keyframe, FrameSE3* &fabMapResult_out, bool includeFABMAP=true, bool closenessTH=1.0);
	FrameSE3* findRePositionCandidate(FrameSE3* frame, float maxScore=1);
	

	inline float getRefFrameScore(float distanceSquared, float usage)
	{
		return distanceSquared*KFDistWeight*KFDistWeight
				+ (1-usage)*(1-usage) * KFUsageWeight * KFUsageWeight;
	}

	float msTrackPermaRef;
	int nTrackPermaRef;
	float nAvgTrackPermaRef;
private:
	/**
	 * Returns a possible loop closure for the keyframe or nullptr if none is found.
	 * Uses FabMap internally.
	 */
	FrameSE3* findAppearanceBasedCandidate(FrameSE3* keyframe);
	std::vector<TrackableKFStructSE3, Eigen::aligned_allocator<TrackableKFStructSE3> > findEuclideanOverlapFrames(FrameSE3* frame, float distanceTH, float angleTH, bool checkBothScales = false);

#ifdef HAVE_FABMAP
	std::unordered_map<int, FrameSE3*> fabmapIDToKeyframe;
	FabMap fabMap;
#endif
	// KeyFrameGraph* graph;
	KeyFrameGraphSE3* graph;

	SE3Tracker* tracker;

	float fowX, fowY;

};

}

#endif
