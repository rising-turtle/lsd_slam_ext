
// #include "DataStructures/FrameMemory.h"
// #include "DataStructures/Frame.h"

#include "DataStructures/FrameMemorySE3.h"
#include "DataStructures/FrameSE3.h"


namespace lsd_slam
{

FrameMemorySE3::FrameMemorySE3()
{
}

FrameMemorySE3& FrameMemorySE3::getInstance()
{
	static FrameMemorySE3 theOneAndOnly;
	return theOneAndOnly;
}

void FrameMemorySE3::releaseBuffes()
{
	boost::unique_lock<boost::mutex> lock(accessMutex);
	int total = 0;


	for(auto p : availableBuffers)
	{
		if(printMemoryDebugInfo)
			printf("deleting %d buffers of size %d!\n", (int)p.second.size(), (int)p.first);

		total += p.second.size() * p.first;

		for(unsigned int i=0;i<p.second.size();i++)
		{
			Eigen::internal::aligned_free(p.second[i]);
			bufferSizes.erase(p.second[i]);
		}

		p.second.clear();
	}
	availableBuffers.clear();

	if(printMemoryDebugInfo)
		printf("released %.1f MB!\n", total / (1000000.0f));
}


void* FrameMemorySE3::getBuffer(unsigned int sizeInByte)
{
	boost::unique_lock<boost::mutex> lock(accessMutex);
	
	if (availableBuffers.count(sizeInByte) > 0)
	{
		std::vector< void* >& availableOfSize = availableBuffers.at(sizeInByte);
		if (availableOfSize.empty())
		{
			void* buffer = allocateBuffer(sizeInByte);
//			assert(buffer != 0);
			return buffer;
		}
		else
		{
			void* buffer = availableOfSize.back();
			availableOfSize.pop_back();

//			assert(buffer != 0);
			return buffer;
		}
	}
	else
	{
		void* buffer = allocateBuffer(sizeInByte);
//		assert(buffer != 0);
		return buffer;
	}
}

float* FrameMemorySE3::getFloatBuffer(unsigned int size)
{
	return (float*)getBuffer(sizeof(float) * size);
}

void FrameMemorySE3::returnBuffer(void* buffer)
{
	if(buffer==0) return;

	boost::unique_lock<boost::mutex> lock(accessMutex);
	
	unsigned int size = bufferSizes.at(buffer);
	//printf("returnFloatBuffer(%d)\n", size);
	if (availableBuffers.count(size) > 0)
		availableBuffers.at(size).push_back(buffer);
	else
	{
		std::vector< void* > availableOfSize;
		availableOfSize.push_back(buffer);
		availableBuffers.insert(std::make_pair(size, availableOfSize));
	}
}

void* FrameMemorySE3::allocateBuffer(unsigned int size)
{
	//printf("allocateFloatBuffer(%d)\n", size);
	
	void* buffer = Eigen::internal::aligned_malloc(size);
	bufferSizes.insert(std::make_pair(buffer, size));
	return buffer;
}

boost::shared_lock<boost::shared_mutex> FrameMemorySE3::activateFrame(FrameSE3* frame)
{
	boost::unique_lock<boost::mutex> lock(activeFramesMutex);
	if(frame->isActive)
		activeFrames.remove(frame);
	activeFrames.push_front(frame);
	frame->isActive = true;
	return boost::shared_lock<boost::shared_mutex>(frame->activeMutex);
}
void FrameMemorySE3::deactivateFrame(FrameSE3* frame)
{
	boost::unique_lock<boost::mutex> lock(activeFramesMutex);
	if(!frame->isActive) return;
	activeFrames.remove(frame);

	while(!frame->minimizeInMemory())
        {
          printf("cannot deactivateFrameSE3 frame %d, as some acvite-lock is lingering. May cause deadlock!\n", frame->id());	// do it in a loop, to make shure it is really, really deactivated.
          return ;
        }

	frame->isActive = false;
}
void FrameMemorySE3::pruneActiveFrames()
{
	boost::unique_lock<boost::mutex> lock(activeFramesMutex);

	while((int)activeFrames.size() > maxLoopClosureCandidates + 20)
	{
		if(!activeFrames.back()->minimizeInMemory())
		{
			if(!activeFrames.back()->minimizeInMemory())
			{
				printf("failed to minimize frame %d twice. maybe some active-lock is lingering?\n",activeFrames.back()->id());
				return;	 // pre-emptive return if could not deactivate.
			}
		}
		activeFrames.back()->isActive = false;
		activeFrames.pop_back();
	}
}

}
