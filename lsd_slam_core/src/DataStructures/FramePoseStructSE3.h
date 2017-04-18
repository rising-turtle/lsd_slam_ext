/*
 * July. 4, 2016, David Z
 *
 *  FramePoseStructSE3 structure to support SE3
 *
 * */

#ifndef FRAME_POSE_STRUCTURE_SE3_H
#define FRAME_POSE_STRUCTURE_SE3_H

#include "util/SophusUtil.h"
// #include "GlobalMapping/g2oTypeSim3Sophus.h"
#include "GlobalMapping/g2oTypeSE3Sophus.h"

namespace lsd_slam
{
class FrameSE3;
class FramePoseStructSE3 {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	FramePoseStructSE3(FrameSE3* frame);
	virtual ~FramePoseStructSE3();

	// parent, the frame originally tracked on. never changes.
	FramePoseStructSE3* trackingParent;

	// set initially as tracking result (then it's a SE(3)),
	// and is changed only once, when the frame becomes a KF (->rescale).
	// Sim3 thisToParent_raw;
        SE3 thisToParent_raw;

	int frameID;
	// Frame* frame;
        FrameSE3* frame;

	// whether this poseStruct is registered in the Graph. if true MEMORY WILL BE HANDLED BY GRAPH
	bool isRegisteredToGraph;

	// whether pose is optimized (true only for KF, after first applyPoseGraphOptResult())
	bool isOptimized;

	// true as soon as the vertex is added to the g2o graph.
	bool isInGraph;

	// graphVertex (if the frame has one, i.e. is a KF and has been added to the graph, otherwise 0).
	// VertexSim3* graphVertex;
        VertexSE3* graphVertex; 
            
	// void setPoseGraphOptResult(Sim3 camToWorld);
        void setPoseGraphOptResult(SE3 camToWorld);
	void applyPoseGraphOptResult();
	// Sim3 getCamToWorld(int recursionDepth = 0);
        SE3 getCamToWorld(int recursionDepth = 0);
	void invalidateCache();
private:
	int cacheValidFor;
	static int cacheValidCounter;

	// absolute position (camToWorld).
	// can change when optimization offset is merged.
	// Sim3 camToWorld;
        SE3 camToWorld;

	// new, optimized absolute position. is added on mergeOptimization.
	// Sim3 camToWorld_new;
        SE3 camToWorld_new;

	// whether camToWorld_new is newer than camToWorld
	bool hasUnmergedPose;
};

}
#endif
