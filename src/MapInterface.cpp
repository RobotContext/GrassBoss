/*
 * MapInterface.cpp
 *
 *  Created on: 1 Dec 2012
 *      Author: kent
 */

#include "MapInterface.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "mapInterface");

	MapInterface mapInterface;
	mapInterface.makeItSpin();

	return 0;
}

MapInterface::MapInterface() :	nodeHandle("~"),
								imageTransporter(this->nodeHandle)
{
	this->setupParameters();
	ROS_INFO("\n"
			"Image file:  %s\n"
			"Image topic: %s\n"
			"Image link:  %s\n"
			, this->imageFile.c_str(), this->imageTopic.c_str(), this->mapLink.c_str());

	//	Setup subscriber
	this->odometrySubscriber = this->nodeHandle.subscribe(this->odometryTopic, 10, &MapInterface::odometryCallback, this);

	//	Setup output and publisher
	this->output.header.frame_id = this->mapLink;
	this->output.encoding = sensor_msgs::image_encodings::MONO8;
	this->imagePublisher = this->imageTransporter.advertise(this->imageTopic, 10);

	//	Load map
	ROS_INFO("\nLoading map...\n");
	this->image = cv::imread(this->imageFile, CV_LOAD_IMAGE_GRAYSCALE);

	this->drawCircle(-10,-10,100,255);

	//	Calculate free space, draw and stuff
	this->qFreeSpace = this->countFreeSpacePixels();
}

MapInterface::~MapInterface()
{
	// TODO Auto-generated destructor stub
}

void MapInterface::setupParameters (void)
{
	this->nodeHandle.param<std::string>("imageFile", this->imageFile, "world/lawn_map_project.pgm");
	this->nodeHandle.param<std::string>("mapLink", this->mapLink, "odom");
	this->nodeHandle.param<std::string>("imageTopic", this->imageTopic, "/mapInterface/map");
	this->nodeHandle.param<std::string>("odometryTopic", this->odometryTopic, "/LIDAR/simulatedOdometry");
	this->nodeHandle.param<double>("scale", this->scale, 0.20f);		//	Each pixel equal 0.20m
}

void MapInterface::makeItSpin (void)
{
	ros::Rate r(10);

	while (ros::ok())
	{
		ros::spinOnce();

		this->output.header.stamp = ros::Time::now();
		this->output.image = this->image;
		this->imagePublisher.publish(this->output.toImageMsg());

		r.sleep();
	}
}

void MapInterface::odometryCallback (const nav_msgs::Odometry::ConstPtr& data)
{
	double x = (double)data.get()->pose.pose.position.x;
	double y = (double)data.get()->pose.pose.position.y;

	x *= this->scale;
	y *= this->scale;

	this->drawCircle(x, y, 50, 50);
}

unsigned int MapInterface::countFreeSpacePixels (void)
{
	unsigned int count(0);

	int width = this->image.cols;
	int height = this->image.rows;

	for (int x = 0; x < width; x++) for (int y = 0; y < height; y++)
		if (this->image.at<unsigned char>(y, x) > 50) count++;

	return count;
}

double MapInterface::freeSpaceCovered (void)
{
	return 1 - ((double)this->countFreeSpacePixels() / (double)this->qFreeSpace);
}

void MapInterface::drawCircle (int x, int y, int radius, unsigned char color)
{
	cv::circle(this->image, cv::Point(x, y), radius, color, -1, CV_AA, 1);
}
