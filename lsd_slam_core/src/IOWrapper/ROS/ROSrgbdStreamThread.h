/*
 *  July. 3, 2016, David Z 
 *  
 *  ROS rgbd stream 
 *
 * */


#ifndef ROSRGBDSTREAM_THREAD_H
#define ROSRGBDSTREAM_THREAD_H

#include "IOWrapper/NotifyBuffer.h"
#include "IOWrapper/TimestampedObject.h"
#include "IOWrapper/InputImageStream.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>

#include "util/Undistorter.h"

namespace lsd_slam
{

const static double timeOverlap = 33.33333333; 

class ROSrgbdStreamThread 
{
public:
    ROSrgbdStreamThread();
    virtual ~ROSrgbdStreamThread(); 

    /**
     * Starts the thread.
     */
    virtual void run();

    void setCalibration_rgb(std::string file);
    void setCalibration_dpt(std::string file);

    inline NotifyBuffer<TimestampedMat2>* getBuffer() {return rgbdBuffer;};

    /**
     * Thread main function.
     */
    void operator()();

    // get called on ros-message callbacks
    void r_vidCb(const sensor_msgs::ImageConstPtr img);
    void r_infoCb(const sensor_msgs::CameraInfoConstPtr info);
 
    void d_vidCb(const sensor_msgs::ImageConstPtr img);
    void d_infoCb(const sensor_msgs::CameraInfoConstPtr info);

    // clear buffer 
    void r_clearBuffer(); 
    void d_clearBuffer();
    void clearBuffer(){r_clearBuffer(); d_clearBuffer();}

    inline float fx() {return fx_;}
    inline float fy() {return fy_;}
    inline float cx() {return cx_;}
    inline float cy() {return cy_;}
    inline int width() {return width_;}
    inline int height() {return height_;}

protected:
    NotifyBuffer<TimestampedMat> * rBuffer; 
    NotifyBuffer<TimestampedMat> * dBuffer;
    NotifyBuffer<TimestampedMat2>* rgbdBuffer; 
    float fx_, fy_, cx_, cy_; // color 
    int width_, height_; 
    
    float d_fx_, d_fy_, d_cx_, d_cy_; // dpt
    int d_width_, d_height_; 

private:

	bool r_haveCalib;
        bool d_haveCalib; 
	Undistorter* r_undistorter;
        Undistorter* d_undistorter; 

	ros::NodeHandle nh_;

	std::string img_channel;
	ros::Subscriber img_sub;
        
        std::string dpt_channel; 
        ros::Subscriber dpt_sub;

	int r_lastSEQ;
        int d_lastSEQ;
};

}

#endif
