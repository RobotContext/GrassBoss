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

	//	Setup subscriber
	this->odometrySubscriber = this->nodeHandle.subscribe(this->odometryTopic, 10, &MapInterface::odometryCallback, this);
	this->resetSubscriber = this->nodeHandle.subscribe("/mapInterface/reset", 10, &MapInterface::resetCallback, this);

	//	Setup output and publisher
	this->output.header.frame_id = this->mapLink;
	this->output.encoding = sensor_msgs::image_encodings::MONO8;
	this->imagePublisher = this->imageTransporter.advertise(this->imageTopic, 10);

	this->coveragePublisher = this->nodeHandle.advertise<std_msgs::Float64>(this->coverageTopic, 10);

	//	Load map
	ROS_INFO("\nLoading map...\n");
	this->image = cv::imread(this->imageFile, CV_LOAD_IMAGE_GRAYSCALE);

	//	Calculate free space, draw and stuff
	this->qFreeSpace = this->countFreeSpacePixels();

	//	Show info
	this->showInfo();
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
	this->nodeHandle.param<std::string>("coverageTopic", this->coverageTopic, "/mapInterface/coverage");
	this->nodeHandle.param<std::string>("odometryTopic", this->odometryTopic, "/LIDAR/simulatedOdometry");
	this->nodeHandle.param<double>("scale", this->scale, 0.20);		//	Each pixel equal 0.20m
}

void MapInterface::showInfo (void)
{
	ROS_INFO(	"\n"
				"Image file:            %s\n"
				"Image width:           %d\n"
				"Image height:          %d\n"
				"Image scale:           Each pixel is %.2fm\n\n"
				"Free space (pixels):   %d\n"
				"Covered (pixels):      %d\n"
				"Coverage percentage:   %.2f%%\n",
			this->imageFile.c_str(), this->image.cols, this->image.rows, this->scale,
			this->qFreeSpace, this->qFreeSpace - this->countFreeSpacePixels(), this->freeSpaceCovered() * 100);
}

void MapInterface::makeItSpin (void)
{
	ros::Rate r(100);

	while (ros::ok())
	{
		ros::spinOnce();

		//	Publish image
		this->output.image = this->image;
		this->output.header.stamp = ros::Time::now();
		this->imagePublisher.publish(this->output.toImageMsg());

		//	Publish coverage
		this->coverage.data = this->freeSpaceCovered();
		this->coveragePublisher.publish(this->coverage);

		r.sleep();
	}
}

void MapInterface::odometryCallback (const nav_msgs::Odometry::ConstPtr& data)
{
	double x = (double)data.get()->pose.pose.position.x;
	double y = (double)data.get()->pose.pose.position.y;

	this->drawCircle((int)(x / this->scale), (int)(y / this->scale), 10, 50);
}

void MapInterface::resetCallback (const geometry_msgs::TwistStamped::ConstPtr& data)
{
	ROS_INFO("\nLoading map...\n");
	this->image = cv::imread(this->imageFile, CV_LOAD_IMAGE_GRAYSCALE);

	//	Calculate free space, draw and stuff
	this->qFreeSpace = this->countFreeSpacePixels();

	//	Show info
	this->showInfo();
}

unsigned int MapInterface::countFreeSpacePixels (void)
{
	unsigned int count(0);

	for (int x = 0; x < this->image.cols; x++)
		for (int y = 0; y < this->image.rows; y++)
			if (this->image.at<unsigned char>(y, x) > 50) count++;

	return count;
}

double MapInterface::freeSpaceCovered (void)
{
	return 1 - ((double)this->countFreeSpacePixels() / (double)this->qFreeSpace);
}

void MapInterface::drawCircle (int x, int y, int radius, unsigned char color)
{
	cv::circle(this->image, 2 * cv::Point(this->image.cols / 2 + x, this->image.rows / 2 - y), radius, color, -1, CV_AA, 1);
}
