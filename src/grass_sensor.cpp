#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include <visualization_msgs/Marker.h>
#include "GrassBoss/angle.h"

#define NUM_OF_MEASUREMENTS 10

ros::Publisher pub;
std_msgs::Bool pub_msg;
float treshold,x_pos,y_pos;

void callback(const std_msgs::Float64::ConstPtr& msg)
{
	static float grass_measurements[NUM_OF_MEASUREMENTS];
	static float grass_last, last_x, last_y;
	static int pointer;

	// if the robot moved since last measurement
	if(x_pos != last_x || y_pos != last_y)
	{
		
		// save measurement
		float value = msg->data - grass_last;
		grass_last = msg->data;
		if( ++pointer == NUM_OF_MEASUREMENTS )
			pointer = 0;
		grass_measurements[pointer] = value;
		last_x = x_pos;
		last_y = y_pos;	
	
		// calculate average and compare to treshold
		value = 0;	
		for( int i = 0 ; i < NUM_OF_MEASUREMENTS ; i++)
			value += grass_measurements[i];
	
		if( value/NUM_OF_MEASUREMENTS > treshold)
		{
			if( ! pub_msg.data )
			{
				pub_msg.data = true;
				pub.publish(pub_msg);
			}
		}
		else
		{
			if( pub_msg.data )
			{
				ROS_WARN("Out of grass. %d",value/NUM_OF_MEASUREMENTS);
				pub_msg.data = false;
				pub.publish(pub_msg);
			}
		}
		
			
	}	
}

void odometry_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
	x_pos = msg->pose.pose.position.x;
	y_pos = msg->pose.pose.position.y;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "grass_sensor");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	std::string sub_topic_id;
	std::string pub_topic_id;
	double trsh;

	n.param<std::string>("sub_topic", sub_topic_id,"/mapInterface/coverage");
	n.param<std::string>("pub_topic", pub_topic_id,"/grass_sensor");
	n.param<double>("treshold", trsh, 0.00005);
	treshold = trsh;

	ros::Subscriber sub = nh.subscribe(sub_topic_id, 10, callback);
	ros::Subscriber odom = nh.subscribe("/odom", 10, odometry_cb);
	pub = nh.advertise<std_msgs::Bool>(pub_topic_id, 1);

	ros::spin();
	return 0;
}

