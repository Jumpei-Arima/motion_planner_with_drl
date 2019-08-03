#include "ros/ros.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

class DemoNavigator{
	public:
		DemoNavigator();
		void PoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
	private:
		ros::NodeHandle nh;
		ros::NodeHandle private_nh;

		//subscriber
		ros::Subscriber pose_sub;

		//publisher
		ros::Publisher local_goal_pub;

		geometry_msgs::PoseWithCovarianceStamped current_pose;
		static const float goals[][2];
		float tolerance;
		int goal_count;
		bool finish;

		double normalize(double z);
		double angle_diff(double a, double b);
		double get_yaw(geometry_msgs::Quaternion q);
};

const float DemoNavigator::goals[3][2] = {
	{ 16.999, 6.716},
	{-16.200, 8.196},
	{  0.516, 0.112}
};

DemoNavigator::DemoNavigator()
	: private_nh("~")
{
	//subscriber
	pose_sub = nh.subscribe("/amcl_pose",1, &DemoNavigator::PoseCallback, this);

	//publisher
	local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal",1,true);
	tolerance = 1.0; //[m]
	goal_count = 0;
	finish = false;
}

void DemoNavigator::PoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	current_pose = *msg;
	float dx = -current_pose.pose.pose.position.x + goals[goal_count][0];
	float dy = -current_pose.pose.pose.position.y + goals[goal_count][1];
	float dis = sqrt(dx*dx + dy*dy);
	if(dis < tolerance){
		std::cout << "goal!!!!!!!!" << std::endl;
		goal_count += 1;
		if(goal_count > sizeof(goals)/sizeof(*goals)){
			finish = true;
		}
	}
	geometry_msgs::PoseStamped local_goal;
	if(!finish){
		local_goal.pose.position.x = dx;
		local_goal.pose.position.y = dy;
		float yaw = angle_diff(atan2(dy,dx), get_yaw(current_pose.pose.pose.orientation));
		local_goal.pose.orientation.x = 0;
		local_goal.pose.orientation.y = 0;
		local_goal.pose.orientation.z = sin(yaw/2);
		local_goal.pose.orientation.w = cos(yaw/2);
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
