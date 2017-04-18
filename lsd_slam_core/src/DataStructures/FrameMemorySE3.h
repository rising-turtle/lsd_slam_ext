#ifndef FRAME_MEMORY_SE3_H
#define FRAME_MEMORY_SE3_H

#include <unordered_map>
#include <vector>
#include <boost/thread/mutex.hpp>
#include <deque>
#include <list>
#include <boost/thread/shared_mutex.hpp>
#include <Eigen/Core> //For EIGEN MACRO

namespace lsd_slam
{

/** Singleton class for re-using buffers in the Frame class. */
class FrameSE3;
class FrameMemorySE3
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/** Returns the global instance. Creates it when the method is first called. */
	static FrameMemorySE3& getInstance();

	/** Allocates or fetches a buffer with length: size * sizeof(float).
	  * Corresponds to "buffer = new float[size]". */
	float* getFloatBuffer(unsigned int size);

	/** Allocates or fetches a buffer with length: size * sizeof(float).
	  * Corresponds to "buffer = new float[size]". */
	void* getBuffer(unsigned int sizeInByte);
	
	/** Returns an allocated buffer back to the global storage for re-use.
	  * Corresponds to "delete[] buffer". */
	void returnBuffer(void* buffer);
	

	boost::shared_lock<boost::shared_mutex> activateFrame(FrameSE3* frame);
	void deactivateFrame(FrameSE3* frame);
	void pruneActiveFrames();

	void releaseBuffes();
private:
	FrameMemorySE3();
	void* allocateBuffer(unsigned int sizeInByte);
	
	boost::mutex accessMutex;
	std::unordered_map< void*, unsigned int > bufferSizes;
	std::unordered_map< unsigned int, std::vector< void* > > availableBuffers;


	boost::mutex activeFramesMutex;
	std::list<FrameSE3*> activeFrames;
};

}

#endif
