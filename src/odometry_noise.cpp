#include "odometry_noise.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "OdometryNoiseSimulation");

  OdometryNoise odometryNode;
  odometryNode.spin();

  ros::spin();

  return 0;
}

void OdometryNoise::setupParameters(void)
{
  this->n.param<std::string>("publisherTopic", this->publisherTopic, "/cmd_vel");
  this->n.param<std::string>("subscriberTopic", this->subscriberTopic, "/velocity");
  this->n.param<double>("noiseMean", this->noiseMean, 0.0);
  this->n.param<double>("noiseDeviation", this->noiseDeviation, 0.05);
}
void OdometryNoise::subscriberCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Read message
  geometry_msgs::Twist noiseMsg(*msg);

  // Apply noise
  noiseMsg.angular.z += (*(this->norm_rnd))();

  // Publish message
  this->pub.publish(noiseMsg);
}
void OdometryNoise::setupNormaldistribution(void)
{
  static int isSetup = 0;
  if (1 == isSetup)
    return;
  isSetup = 1;

  this->rand_generator = new boost::mt19937();
  this->rand_generator->seed(std::time(0));
  this->norm_dist = new boost::normal_distribution<>(this->noiseMean, this->noiseDeviation);
  this->norm_rnd = new boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >(*(this->rand_generator), *(this->norm_dist));
}
OdometryNoise::OdometryNoise() : n("~")
{
  // Parameters
  this->setupParameters();
  this->setupNormaldistribution();
  
  // Publisher and subscriber topics set up
  this->pub = this->n.advertise<geometry_msgs::Twist>(this->publisherTopic, 10);
  this->sub = this->n.subscribe(this->subscriberTopic, 10, &OdometryNoise::subscriberCallback, this);
  

}
void OdometryNoise::spin(void)
{

}
