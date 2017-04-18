
/*  July. 4, 2016 David Z 
 *  
 *  A graph structure consisted by SE3Sophus 
 *
 * */

#ifndef KEYFRAME_GRAPH_SE3_H
#define KEYFRAME_GRAPH_SE3_H

#include <vector>
#include <unordered_map>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include "util/EigenCoreInclude.h"
#include <g2o/core/sparse_optimizer.h>
#include "util/SophusUtil.h"
#include "deque"


namespace lsd_slam
{


class Frame;
class FrameSE3;
class KeyFrameGraph;
class VertexSim3;
class EdgeSim3;
class FramePoseStruct;
class FramePoseStructSE3;
class VertexSE3; 
class EdgeSE3;

struct KFConstraintStructSE3
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	inline KFConstraintStructSE3()
	{
		firstFrame = secondFrame = 0;
		information.setZero();
		robustKernel = 0;
		edge = 0;

		usage = meanResidual = meanResidualD = meanResidualP = 0;
		reciprocalConsistency = 0;


		idxInAllEdges = -1;
	}

	~KFConstraintStructSE3();


	FrameSE3* firstFrame;
	FrameSE3* secondFrame;
	// Sophus::Sim3d secondToFirst;
        Sophus::SE3d secondToFirst;
	// Eigen::Matrix<double, 7, 7> information;
        Eigen::Matrix<double, 6, 6> information; 
	g2o::RobustKernel* robustKernel;
	// EdgeSim3* edge;
        EdgeSE3* edge;

	float usage;
	float meanResidualD;
	float meanResidualP;
	float meanResidual;

	float reciprocalConsistency;

	int idxInAllEdges;
};






/**
 * Graph consisting of KeyFrames and constraints, performing optimization.
 */
class KeyFrameGraphSE3
{
friend class IntegrationTest;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/** Constructs an empty pose graph. */
	KeyFrameGraphSE3();
	
	/** Deletes the g2o graph. */
	~KeyFrameGraphSE3();
	
	/** Adds a new KeyFrame to the graph. */
	void addKeyFrame(FrameSE3* frame);
	
	/** Adds a new Frame to the graph. Doesnt actually keep the frame, but only it's pose-struct. */
	void addFrame(FrameSE3* frame);

	void dumpMap(std::string folder);

        void saveGraph(std::string file);

	/**
	 * Adds a new constraint to the graph.
	 * 
	 * The transformation must map world points such that they move as if
	 * attached to a frame which moves from firstFrame to secondFrame:
	 * second->camToWorld * first->worldToCam * point
	 * 
	 * If isOdometryConstraint is set, scaleInformation is ignored.
	 */
	void insertConstraint(KFConstraintStructSE3* constraint);

	
	/** Optimizes the graph. Does not update the keyframe poses,
	 *  only the vertex poses. You must call updateKeyFramePoses() afterwards. */
	int optimize(int num_iterations);
	bool addElementsFromBuffer();

	
	/**
	 * Creates a hash map of keyframe -> distance to given frame.
	 */
	void calculateGraphDistancesToFrame(FrameSE3* frame, std::unordered_map<FrameSE3*, int>* distanceMap);
	


	int totalPoints;
	int totalEdges;
	int totalVertices;


	//=========================== Keyframe & Posen Lists & Maps ====================================
	// Always lock the list with the corresponding mutex!
	// central point to administer keyframes, iterate over keyframes, do lookups etc.


	// contains ALL keyframes, as soon as they are "finished".
	// does NOT yet contain the keyframe that is currently being created.
	boost::shared_mutex keyframesAllMutex;
	std::vector< FrameSE3*, Eigen::aligned_allocator<FrameSE3*> > keyframesAll;


	/** Maps frame ids to keyframes. Contains ALL Keyframes allocated, including the one that currently being created. */
	/* this is where the shared pointers of Keyframe Frames are kept, so they are not deleted ever */
	boost::shared_mutex idToKeyFrameMutex;
	std::unordered_map< int, std::shared_ptr<FrameSE3>, std::hash<int>, std::equal_to<int>,
	Eigen::aligned_allocator< std::pair<const int, std::shared_ptr<FrameSE3> > > > idToKeyFrame;


	// contains ALL edges, as soon as they are created
	boost::shared_mutex edgesListsMutex;
	std::vector< KFConstraintStructSE3*, Eigen::aligned_allocator<KFConstraintStructSE3*> > edgesAll;



	// contains ALL frame poses, chronologically, as soon as they are tracked.
	// the corresponding frame may have been removed / deleted in the meantime.
	// these are the ones that are also referenced by the corresponding Frame / Keyframe object
	boost::shared_mutex allFramePosesMutex;
	std::vector<FramePoseStructSE3*, Eigen::aligned_allocator<FramePoseStructSE3*> > allFramePoses;

	// contains all keyframes in graph, in some arbitrary (random) order. if a frame is re-tracked,
	// it is put to the end of this list; frames for re-tracking are always chosen from the first third of
	// this list.
	boost::mutex keyframesForRetrackMutex;
	std::deque<FrameSE3*> keyframesForRetrack;



private:

	/** Pose graph representation in g2o */
	g2o::SparseOptimizer graph;
	
	std::vector< FrameSE3*, Eigen::aligned_allocator<FrameSE3*> > newKeyframesBuffer;
	std::vector< KFConstraintStructSE3*, Eigen::aligned_allocator<KFConstraintStructSE3*> > newEdgeBuffer;


	int nextEdgeId;
};

}

#endif
