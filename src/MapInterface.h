/*
 * MapInterface.h
 *
 *  Created on: 1 Dec 2012
 *      Author: kent
 */

#ifndef MAPINTERFACE_H_
#define MAPINTERFACE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/highgui.h>
#include <nav_msgs/Odometry.h>

class MapInterface
{
private:
	//	Parameters
	std::string imageFile;
	std::string mapLink;
	std::string imageTopic;
	std::string odometryTopic;
	double scale;

	//	ROS
	ros::NodeHandle nodeHandle;
	ros::Subscriber odometrySubscriber;
	image_transport::ImageTransport imageTransporter;
	image_transport::Publisher imagePublisher;
	cv_bridge::CvImage output;

	//	Image matrix
	cv::Mat image;

	//	Info
	unsigned int qFreeSpace;

	//	Functions
	void setupParameters (void);
	unsigned int countFreeSpacePixels (void);
	double freeSpaceCovered (void);
	void drawCircle (int x, int y, int radius, unsigned char color);

	//	Callbacks
	void odometryCallback (const nav_msgs::Odometry::ConstPtr& data);

public:
	MapInterface();
	virtual ~MapInterface();

	void makeItSpin (void);
};

#endif /* MAPINTERFACE_H_ */
