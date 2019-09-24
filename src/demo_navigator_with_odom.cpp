#include "ros/ros.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

class DemoNavigator{
	public:
		DemoNavigator();
		void OdomCallback(const nav_msgs::OdometryConstPtr& msg);
	private:
		ros::NodeHandle nh;
		ros::NodeHandle private_nh;

		//subscriber
		ros::Subscriber pose_sub;

		//publisher
		ros::Publisher local_goal_pub;


		double position_x;
		double position_y;
		double orientation_yaw;
		std::vector<std::vector<float>> goals;
		float tolerance;
		int goal_count;
		bool finish;
		bool odom_callback_first;
		std::string ROBOT_FRAME;

		double normalize(double z);
		double angle_diff(double a, double b);
		double get_yaw(geometry_msgs::Quaternion q);
		ros::Time last_time;
};

DemoNavigator::DemoNavigator()
	: private_nh("~")
{
	//subscriber
	pose_sub = nh.subscribe("/odom",1, &DemoNavigator::OdomCallback, this);

	//publisher
	local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal",1,true);
	position_x = 0.0;
	position_y = 0.0;
	orientation_yaw = 0.0;
	tolerance = 1.0; //[m]
	goal_count = 0;
	finish = false;
	odom_callback_first = true;
	goals = {
		{ 16.999, 6.716},
		{-16.200, 8.196},
		{  0.516, 0.112}
	};

}

void DemoNavigator::OdomCallback(const nav_msgs::OdometryConstPtr& msg)
{

	geometry_msgs::PoseStamped local_goal;
	local_goal.header.stamp = ros::Time::now();
	local_goal.header.frame_id = ROBOT_FRAME;
	if(!finish){
		// calc odom
		nav_msgs::Odometry odom = *msg;
		ros::Time current_time = odom.header.stamp;
		if(odom_callback_first){
			last_time = current_time;
			odom_callback_first = false;
		}
		double dt = (current_time-last_time).toSec();
		last_time = current_time;
		double dist = odom.twist.twist.linear.x*dt;
		orientation_yaw += odom.twist.twist.angular.z*dt;
		position_x += dist * cos(orientation_yaw);
		position_y += dist * sin(orientation_yaw);
		
		//calc relative goal(local_goal)
		float dx = -position_x + goals[goal_count][0];
		float dy = -position_y + goals[goal_count][1];
		float dis = sqrt(dx*dx + dy*dy);
		if(dis < tolerance){
			std::cout << "goal!!!!!!!!" << std::endl;
			goal_count += 1;
			if(goal_count == goals.size()){
				finish = true;
			}
		}
		float yaw = angle_diff(atan2(dy,dx), orientation_yaw);
		local_goal.pose.position.x = dis*cos(yaw);
		local_goal.pose.position.y = dis*sin(yaw);
		local_goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		std::cout << "goal count "<< goal_count << std::endl;
	}else{
		std::cout << "========finish========" << std::endl;
		local_goal.pose.position.x = 0.0;
		local_goal.pose.position.y = 0.0;
		local_goal.pose.orientation.x = 0;
		local_goal.pose.orientation.y = 0;
		local_goal.pose.orientation.z = 0;
		local_goal.pose.orientation.w = 1;
	}
	std::cout << "========local goal========" << std::endl;
	std::cout << local_goal << std::endl;
	local_goal_pub.publish(local_goal);
}

double DemoNavigator::normalize(double z)
{
	return atan2(sin(z),cos(z));
}
double DemoNavigator::angle_diff(double a, double b)
{
	double d1, d2;
	a = normalize(a);
	b = normalize(b);
	d1 = a-b;
	d2 = 2*M_PI - fabs(d1);
	if(d1 > 0)
		d2 *= -1.0;
	if(fabs(d1) < fabs(d2))
		return(d1);
	else
		return(d2);
}

double DemoNavigator::get_yaw(geometry_msgs::Quaternion q)
{
	double r, p, y;
	tf::Quaternion quaternion(q.x, q.y, q.z, q.w);
	tf::Matrix3x3(quaternion).getRPY(r, p, y);
	return y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_navigator");

	DemoNavigator demo_navigator;

	ros::spin();
    return 0;
}
