#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>

#include "GrassBoss/angle.h"

ros::Publisher angle_pub;
ros::Publisher marker_pub;
GrassBoss::angle angle_msg;
visualization_msgs::Marker marker;
double  max_value = 5.0,
		left_offset_x = -0.4,
		left_offset_y = 0.2,
		right_offset_x = -0.4,
		right_offset_y = -0.2;

double max( double a , double b )
{
	if( a > b )
		return a;
	else
		return b;
}

double min( double a , double b )
{
	if( a < b )
		return a;
	else
		return b;
}
void scanner_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	double pt_x , pt_y , left_min , right_min;
	std::vector< double > left_dist , right_dist;

	unsigned int min_index , max_index;
	float low_scan_angle , high_scan_angle;
	double pt_a_x , pt_a_y , pt_b_x , pt_b_y , dx ,dy;

	angle_msg.header.stamp = msg->header.stamp;

	// Iterate through points
	for( unsigned int i = 0 ; i < msg->ranges.size() ; i++ )
	{
		// Transform from polar coordinates to euclidian coordinates
		pt_x = msg->ranges.at(i) * cos( msg->angle_min + ( i * msg->angle_increment ) );
		pt_y = msg->ranges.at(i) * sin( msg->angle_min + ( i * msg->angle_increment ) );

		// Offset points to represent different sensors and calculate distance
		left_dist.push_back( sqrt( pow( (pt_x + left_offset_x),2 ) + pow( (pt_y + left_offset_y),2 ) ) );
		right_dist.push_back( sqrt( pow( (pt_x + right_offset_x),2 ) + pow( (pt_y + right_offset_y),2 ) ) );
	}

	// Find minimum distance in sensor lists
	left_min = *(left_dist.begin());
	for( std::vector<double>::iterator itr = left_dist.begin() ; itr != left_dist.end() ; itr++ )
		if( *itr < left_min )
			left_min = *itr;

	right_min = *(right_dist.begin());
	for( std::vector<double>::iterator itr = right_dist.begin() ; itr != right_dist.end() ; itr++ )
		if( *itr < right_min )
			right_min = *itr;

	// Add minimums to message if distance is within range
	angle_msg.left = left_min < max_value ? left_min : max_value;
	angle_msg.right = right_min < max_value ? right_min : max_value;

	// Find minimum distance to both sides
	for( min_index = 0 ; min_index < msg->ranges.size() && msg->ranges.at(min_index) == msg->range_max ; min_index++ );
	for( max_index = msg->ranges.size() - 1 ; max_index > 0  && msg->ranges.at(max_index) == msg->range_max ; max_index-- );

	// If an actual minimum was found
	if( min_index < max_index )
	{
		// Transform polar vectors to point
		low_scan_angle = msg->angle_min + ( min_index * msg->angle_increment );
		pt_a_x = msg->ranges.at(min_index) * cos( low_scan_angle );
		pt_a_y = msg->ranges.at(min_index) * sin( low_scan_angle );
		high_scan_angle = msg->angle_min + ( max_index * msg->angle_increment );
		pt_b_x = msg->ranges.at(max_index) * cos( high_scan_angle );
		pt_b_y = msg->ranges.at(max_index) * sin( high_scan_angle );

		// Calculate range of x and y
		dx = max(pt_a_x , pt_b_x) - min(pt_a_x , pt_b_x);
		dy = max(pt_a_y , pt_b_y) - min(pt_a_y , pt_b_y);

		// Calculate angle relative to robot
		angle_msg.angle = 360 * ( atan(dx/dy) / 6.28 );

		// Sign angle
		if( msg->ranges.at(min_index) > msg->ranges.at(max_index) )
			angle_msg.angle *= -1;

		// Generate header for rviz marker
		marker.header.frame_id = "base_laser_link";
		marker.header.stamp = msg->header.stamp;
		marker.ns = "grassboss";
		marker.pose.orientation.w = 1.0;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.id = 1;

		// Set size and color of marker
		marker.scale.x = 0.2;
		marker.color.b = 1.0;
		marker.color.r = 0;
		marker.color.a = 0.6;

		// Set points
		geometry_msgs::Point p;
		marker.points.clear();
		p.x = pt_a_x;
		p.y = pt_a_y;
		p.z = 0;
		marker.points.push_back(p);
		p.x = pt_b_x;
		p.y = pt_b_y;
		marker.points.push_back(p);

		// Publish marker
		marker_pub.publish(marker);
	}
	else
		// If no minimum was found
		angle_msg.angle = 0;

	// Publish message
	angle_pub.publish(angle_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "angle_parser");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	std::string scan_topic_id;
	std::string angle_topic_id;

	n.param<std::string>("scanner_topic", scan_topic_id,
			"/base_scan");
	n.param<std::string>("angle_topic", angle_topic_id,
			"/angle");

	ros::Subscriber scan_sub = nh.subscribe(scan_topic_id, 10, scanner_callback);
	angle_pub = nh.advertise<GrassBoss::angle>(angle_topic_id, 1);
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	ros::spin();
	return 0;
}

