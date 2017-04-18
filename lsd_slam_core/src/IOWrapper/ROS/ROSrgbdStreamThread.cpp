
#include "ROSrgbdStreamThread.h"
#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "cv_bridge/cv_bridge.h"
#include "util/settings.h"

#include <iostream>
#include <fstream>

namespace lsd_slam
{
  ROSrgbdStreamThread::ROSrgbdStreamThread()
  {
    // subscribe 
    img_channel = nh_.resolveName("color_image");
    img_sub = nh_.subscribe(img_channel, 1, &ROSrgbdStreamThread::r_vidCb, this); 
    
    dpt_channel = nh_.resolveName("depth_image"); 
    dpt_sub = nh_.subscribe(dpt_channel, 1, &ROSrgbdStreamThread::d_vidCb, this);

    // wait for cam calib 
    width_ = height_ = d_width_ = d_height_ = 0; 

    rBuffer = new NotifyBuffer<TimestampedMat>(8); 
    dBuffer = new NotifyBuffer<TimestampedMat>(8); 
    rgbdBuffer = new NotifyBuffer<TimestampedMat2>(8); 
    
    r_undistorter = d_undistorter = 0;
    r_lastSEQ = d_lastSEQ = 0; 
    
    r_haveCalib = d_haveCalib = false; 
  }
  
  ROSrgbdStreamThread::~ROSrgbdStreamThread()
  {
    delete rBuffer; 
    delete dBuffer; 
    delete rgbdBuffer; 
  }
  
  void ROSrgbdStreamThread::setCalibration_rgb(std::string file)
  {
  	if(file == "")
	{
		ros::Subscriber info_sub  = nh_.subscribe(nh_.resolveName("rgb/camera_info"),1, &ROSrgbdStreamThread::r_infoCb, this);

		printf("WAITING for ROS camera calibration!\n");
		while(width_ == 0)
		{
			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.03));
		}
		printf("RECEIVED ROS camera calibration!\n");

		info_sub.shutdown();
	}
	else
	{
		r_undistorter = Undistorter::getUndistorterForFile(file.c_str());

		if(r_undistorter==0)
		{
			printf("Failed to read camera calibration from file... wrong syntax?\n");
			exit(0);
		}

		fx_ = r_undistorter->getK().at<double>(0, 0);
		fy_ = r_undistorter->getK().at<double>(1, 1);
		cx_ = r_undistorter->getK().at<double>(2, 0);
		cy_ = r_undistorter->getK().at<double>(2, 1);

		width_ = r_undistorter->getOutputWidth();
		height_ = r_undistorter->getOutputHeight();
	}

	r_haveCalib = true;
  }
 
  void ROSrgbdStreamThread::setCalibration_dpt(std::string file)
  {
  	if(file == "")
	{
		ros::Subscriber info_sub  = nh_.subscribe(nh_.resolveName("dpt/camera_info"),1, &ROSrgbdStreamThread::d_infoCb, this);

		printf("WAITING for ROS camera calibration!\n");
		while(d_width_ == 0)
		{
			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.03));
		}
		printf("RECEIVED ROS camera calibration!\n");

		info_sub.shutdown();
	}
	else
	{
		d_undistorter = Undistorter::getUndistorterForFile(file.c_str());

		if(d_undistorter==0)
		{
			printf("Failed to read camera calibration from file... wrong syntax?\n");
			exit(0);
		}

		d_fx_ = d_undistorter->getK().at<double>(0, 0);
		d_fy_ = d_undistorter->getK().at<double>(1, 1);
		d_cx_ = d_undistorter->getK().at<double>(2, 0);
		d_cy_ = d_undistorter->getK().at<double>(2, 1);

		d_width_ = d_undistorter->getOutputWidth();
		d_height_ = d_undistorter->getOutputHeight();
	}

	d_haveCalib = true;
  }
  
void ROSrgbdStreamThread::run()
{
	boost::thread thread(boost::ref(*this));
}

void ROSrgbdStreamThread::operator()()
{
	ros::spin();

	exit(0);
}

void ROSrgbdStreamThread::d_infoCb(const sensor_msgs::CameraInfoConstPtr info)
{
	if(!d_haveCalib)
	{
		d_fx_ = info->P[0];
		d_fy_ = info->P[5];
		d_cx_ = info->P[2];
		d_cy_ = info->P[6];

		if(d_fx_ == 0 || d_fy_==0)
		{
			printf("camera calib from P seems wrong, trying calib from K\n");
			d_fx_ = info->K[0];
			d_fy_ = info->K[4];
			d_cx_ = info->K[2];
			d_cy_ = info->K[5];
		}

		d_width_ = info->width;
		d_height_ = info->height;

		printf("Received ROS DEPTH Camera Calibration: fx: %f, fy: %f, cx: %f, cy: %f @ %dx%d\n",d_fx_,d_fy_,d_cx_,d_cy_,d_width_,d_height_);
                d_haveCalib = true;
	}
}

void ROSrgbdStreamThread::r_infoCb(const sensor_msgs::CameraInfoConstPtr info)
{
	if(!r_haveCalib)
	{
		fx_ = info->P[0];
		fy_ = info->P[5];
		cx_ = info->P[2];
		cy_ = info->P[6];

		if(fx_ == 0 || fy_==0)
		{
			printf("camera calib from P seems wrong, trying calib from K\n");
			fx_ = info->K[0];
			fy_ = info->K[4];
			cx_ = info->K[2];
			cy_ = info->K[5];
		}

		width_ = info->width;
		height_ = info->height;

		printf("Received ROS COLOR Camera Calibration: fx: %f, fy: %f, cx: %f, cy: %f @ %dx%d\n",fx_,fy_,cx_,cy_,width_,height_);
                r_haveCalib = true;
	}
}

void ROSrgbdStreamThread::r_clearBuffer()
{
  while(rBuffer->size() > 0)
  {
    rBuffer->popFront(); 
  }
}

void ROSrgbdStreamThread::d_clearBuffer()
{
  while(dBuffer->size() > 0)
  {
    dBuffer->popFront(); 
  }
}

void ROSrgbdStreamThread::r_vidCb(const sensor_msgs::ImageConstPtr img)
{
  if(!r_haveCalib) return ;
  // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8); 
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGBA8); 

  if(img->header.seq < (unsigned int) r_lastSEQ)
  {
    printf("Backward-Jump in SEQ detected, but ignoring for now \n" ); 
    r_lastSEQ = 0; 
    return ;
  }

  r_lastSEQ = img->header.seq;

  TimestampedMat bufferItem;
  if(img->header.stamp.toSec() != 0)
    bufferItem.timestamp =  Timestamp(img->header.stamp.toSec());
  else
    bufferItem.timestamp =  Timestamp(ros::Time::now().toSec());
  
  if(r_undistorter !=0 )
  {
    assert(r_undistorter->isValid()); 
    r_undistorter->undistort(cv_ptr->image, bufferItem.data); 
  }else
  {
    bufferItem.data = cv_ptr->image; 
  }

  TimestampedMat2 new_bufferItem; 
  // bool b_new_data_good = false;

  // case 1, if the other queue is empty 
  double ref_t = bufferItem.timestamp.toSec() * 1000. ; 
  double pro_t ;
  
  do{
      if(dBuffer->size() == 0)
      {
        rBuffer->pushBack(bufferItem);
        break;
      }

      TimestampedMat dItem = dBuffer->first();
      pro_t = dItem.timestamp.toSec() * 1000.; 
      if(pro_t >= ref_t )  // case 2, the other is newer and 
      {
        if((pro_t - ref_t) < timeOverlap) // satisfy the condition 
        {
          new_bufferItem.timestamp = pro_t; 
          new_bufferItem.data1 = cv_ptr->image; 
          new_bufferItem.data2 = dItem.data;
          dBuffer->popFront(); 
          // ROS_INFO("succeed add a rgbd frame with new rgb data! ");
          rgbdBuffer->pushBack(new_bufferItem); 
        } // the other is much newer than this queue, delete all item in this queue

        r_clearBuffer(); // items in this queue are all old 
        break; 
        
      }else   // case 3, the other is older
      {
        if( (ref_t - pro_t) < timeOverlap) // 
        {
          new_bufferItem.timestamp = ref_t; 
          new_bufferItem.data1 = cv_ptr->image; 
          new_bufferItem.data2 = dItem.data;
          dBuffer->popFront(); 
          r_clearBuffer();  // drop off all the previous buffers
          // ROS_INFO("succeed add a rgbd frame with new rgb data! ");
          rgbdBuffer->pushBack(new_bufferItem);
          break;
        }else
        {
          dBuffer->popFront(); 
          continue;
        }
      }

    }while(dBuffer->size() > 0);

  return ;   
}


void ROSrgbdStreamThread::d_vidCb(const sensor_msgs::ImageConstPtr img)
{
  if(!d_haveCalib) return ;
  // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16); 
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1); 

  if(img->header.seq < (unsigned int) d_lastSEQ)
  {
    printf("Backward-Jump in SEQ detected, but ignoring for now \n" ); 
    d_lastSEQ = 0; 
    return ;
  }

  d_lastSEQ = img->header.seq;

  TimestampedMat bufferItem;
  if(img->header.stamp.toSec() != 0)
    bufferItem.timestamp =  Timestamp(img->header.stamp.toSec());
  else
    bufferItem.timestamp =  Timestamp(ros::Time::now().toSec());
  
  if(d_undistorter != 0)
  {
    assert(d_undistorter->isValid()); 
    d_undistorter->undistort(cv_ptr->image, bufferItem.data); 
  }else
  {
    bufferItem.data = cv_ptr->image;
  }

  TimestampedMat2 new_bufferItem; 
  // bool b_new_data_good = false;

  // case 1, if the other queue is empty 
  double ref_t = bufferItem.timestamp.toSec() * 1000. ; 
  double pro_t ;
  
  do{
      if(rBuffer->size() == 0)
      {
        dBuffer->pushBack(bufferItem);
        break;
      }

      TimestampedMat rItem = rBuffer->first();
      pro_t = rItem.timestamp.toSec() * 1000.; 
      if(pro_t >= ref_t )  // case 2, the other is newer and 
      {
        if((pro_t - ref_t) < timeOverlap) // satisfy the condition 
        {
          new_bufferItem.timestamp = pro_t; 
          new_bufferItem.data1 = rItem.data; // cv_ptr->image; 
          new_bufferItem.data2 = bufferItem.data; // dItem.data;
          rBuffer->popFront(); 
          // ROS_INFO("succeed add a rgbd frame with new depth data! ");
          rgbdBuffer->pushBack(new_bufferItem); 
        } // the other is much newer than this queue, delete all item in this queue

        d_clearBuffer(); // items in this queue are all old 
        break; 
        
      }else   // case 3, the other is older
      {
        if( (ref_t - pro_t) < timeOverlap) // 
        {
          new_bufferItem.timestamp = ref_t; 
          new_bufferItem.data1 = rItem.data; // cv_ptr->image; 
          new_bufferItem.data2 = bufferItem.data; // dItem.data;
          rBuffer->popFront(); 
          d_clearBuffer();  // drop off all the previous buffers
          // ROS_INFO("succeed add a rgbd frame with new depth data! ");
          rgbdBuffer->pushBack(new_bufferItem);
          break;
        }else
        {
          rBuffer->popFront(); 
          continue;
        }
      }

    }while(rBuffer->size() > 0);
  
  return ;
}



}
