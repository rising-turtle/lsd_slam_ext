
// #include <DataStructures/FramePoseStruct.h>
// #include "DataStructures/Frame.h"
#include <DataStructures/FramePoseStructSE3.h>
#include "DataStructures/FrameSE3.h"

namespace lsd_slam
{

int FramePoseStructSE3::cacheValidCounter = 0;


int privateFramePoseSE3StructAllocCount = 0;

FramePoseStructSE3::FramePoseStructSE3(FrameSE3* frame)
{
	cacheValidFor = -1;
	isOptimized = false;
	// thisToParent_raw = camToWorld = camToWorld_new = Sim3();
        thisToParent_raw = camToWorld = camToWorld_new = SE3();
	this->frame = frame;
	frameID = frame->id();
	trackingParent = 0;
	isRegisteredToGraph = false;
	hasUnmergedPose = false;
	isInGraph = false;

	this->graphVertex = nullptr;

	privateFramePoseSE3StructAllocCount++;
	if(enablePrintDebugInfo && printMemoryDebugInfo)
		printf("ALLOCATED pose %d, now there are %d\n", frameID, privateFramePoseSE3StructAllocCount);
}

FramePoseStructSE3::~FramePoseStructSE3()
{
	privateFramePoseSE3StructAllocCount--;
	if(enablePrintDebugInfo && printMemoryDebugInfo)
		printf("DELETED pose %d, now there are %d\n", frameID, privateFramePoseSE3StructAllocCount);
}

void FramePoseStructSE3::setPoseGraphOptResult(SE3 camToWorld)
{
	if(!isInGraph)
		return;


	camToWorld_new = camToWorld;
	hasUnmergedPose = true;
}

void FramePoseStructSE3::applyPoseGraphOptResult()
{
	if(!hasUnmergedPose)
		return;


	camToWorld = camToWorld_new;
	isOptimized = true;
	hasUnmergedPose = false;
	cacheValidCounter++;
}
void FramePoseStructSE3::invalidateCache()
{
	cacheValidFor = -1;
}
SE3 FramePoseStructSE3::getCamToWorld(int recursionDepth)
{
	// prevent stack overflow
	assert(recursionDepth < 5000);

	// if the node is in the graph, it's absolute pose is only changed by optimization.
	if(isOptimized) return camToWorld;


	// return chached pose, if still valid.
	if(cacheValidFor == cacheValidCounter)
		return camToWorld;

	// return id if there is no parent (very first frame)
	if(trackingParent == nullptr)
		// return camToWorld = Sim3();
                return camToWorld = SE3();

	// abs. pose is computed from the parent's abs. pose, and cached.
	cacheValidFor = cacheValidCounter;

	return camToWorld = trackingParent->getCamToWorld(recursionDepth+1) * thisToParent_raw;
}

}
