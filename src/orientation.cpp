#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Empty.h>


#define _USE_MATH_DEFINES
#include <cmath>

ros::Publisher euler_pub,initial_pub,delta_pub;

sensor_msgs::Imu imu;

//initial values of roll, pitch, yaw
double iR,iP,iY;
bool reset = true;

double degrees (double rad)
{
	return rad * 180 / M_PI;
}

void resetInitialValues(double r,double p,double y)
{
	iR = r;
	iP = p;
	iY = y;
	ROS_INFO("Current orientation: %lf %lf %lf",degrees(iR),degrees(iP),degrees(iY));
}

double delta (double initial, double final)
{
	double d = final - initial;
	if (final - initial < 0)
		d += M_PI_2;
	if (d > M_PI)
		d -= M_PI_2;
	return d;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	imu.header = msg->header;
	// Convert quaternion to RPY.
	tf::Quaternion q;
	double r, p, y; // read values
	double dR,dP,dY;// delta orientation
	tf::quaternionMsgToTF(msg->orientation, q);
	tf::Matrix3x3(q).getRPY(r, p, y);

	if (reset){
    	resetInitialValues(r, p, y);
	    geometry_msgs::Vector3 initial;
	    initial.x = degrees(iR);
	    initial.y = degrees(iP);
	    initial.z = degrees(iY);
	    initial_pub.publish(initial);
    	reset = false;

    }
	    dR = delta(iR,r);
	    dP = delta(iP,p);
	    dY = delta(iY,y);

	    geometry_msgs::Vector3 euler;
	    euler.x = degrees(r);
	    euler.y = degrees(p);
	    euler.z = degrees(y);
	    euler_pub.publish(euler);

	    geometry_msgs::Vector3 delta;
	    delta.x = degrees(dR);
	    delta.y = degrees(dP);
	    delta.z = degrees(dY);
	    delta_pub.publish(delta);
}

void resetCallback(const std_msgs::Empty &msg) {
	reset = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "orientation");
  ros::NodeHandle n;

  ros::Subscriber data_sub = n.subscribe("imu/data", 100, imuCallback);
  ros::Subscriber reset_sub = n.subscribe("imu/reset", 1, resetCallback);

  euler_pub = n.advertise<geometry_msgs::Vector3>("imu/euler", 100);
  initial_pub = n.advertise<geometry_msgs::Vector3>("imu/initial", 100);
  delta_pub = n.advertise<geometry_msgs::Vector3>("imu/delta", 100);

  ros::spin();
}
