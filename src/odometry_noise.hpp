#ifndef ODOMETRY_NOICE
#define ODOMETRY_NOICE

#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <boost/random.hpp>

class OdometryNoise
{
private:
  std::string publisherTopic;
  std::string subscriberTopic;
double noiseMean;
double noiseDeviation;
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
  void setupParameters(void);
  void subscriberCallback(const geometry_msgs::Twist::ConstPtr&);
void setupNormaldistribution(void);
// Boost
boost::mt19937 * rand_generator;
boost::normal_distribution<> * norm_dist;
boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > * norm_rnd;
public:
  OdometryNoise();
  void spin(void);
};

#endif
