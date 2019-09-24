#include "ros/ros.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class TestNavigator{
	public:
		TestNavigator();
		void process();
	private:
		ros::NodeHandle nh;
		ros::NodeHandle private_nh;

		//tf
		tf::TransformListener listener;

		//publisher
		ros::Publisher local_goal_pub;

		std::vector<std::vector<float>> goals;
		float tolerance;
		int goal_count;
		bool finish;
		std::string WORLD_FRAME;
		std::string ROBOT_FRAME;
		double HZ;

		double normalize(double z);
		double angle_diff(double a, double b);
		double get_yaw(geometry_msgs::Quaternion q);
};


TestNavigator::TestNavigator()
	: private_nh("~")
{
	//publisher
	local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal",1,true);
	
	WORLD_FRAME = "map";
	ROBOT_FRAME = "base_link";
	HZ = 10;
	tolerance = 0.1; //[m]
	goal_count = 0;
	finish = false;
	goals = {
		{ 10.000, 0.000}
	};
}

void TestNavigator::process()
{
	ros::Rate loop_rate(HZ);
	while(ros::ok()){
		geometry_msgs::PoseStamped local_goal;
		local_goal.header.stamp = ros::Time::now();
		local_goal.header.frame_id = ROBOT_FRAME;
		if(!finish){
			geometry_msgs::PoseStamped pose;
			float dx = 0;
			float dy = 0;
			float dis = 0;
			try{
				tf::StampedTransform robot_transform;
				listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time(0), robot_transform);
				tf::poseStampedTFToMsg(tf::Stamped<tf::Transform>(robot_transform, robot_transform.stamp_, robot_transform.frame_id_), pose);
				dx=goals[goal_count][0] - pose.pose.position.x;
				dy=goals[goal_count][1] - pose.pose.position.y;
				dis = sqrt(dx*dx + dy*dy);
				if(dis < tolerance){
					std::cout << "goal!!!!!!!!" << std::endl;
					goal_count += 1;
					if(goal_count == goals.size()){
						finish = true;
					}
				}
			}catch(tf::TransformException ex){
				std::cout << ex.what() << std::endl;
			}
			float yaw = angle_diff(atan2(dy,dx),get_yaw(pose.pose.orientation));
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
		ros::spinOnce();
		loop_rate.sleep();
	}
}

double TestNavigator::normalize(double z)
{
	return atan2(sin(z),cos(z));
}
double TestNavigator::angle_diff(double a, double b)
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

double TestNavigator::get_yaw(geometry_msgs::Quaternion q)
{
	double r, p, y;
	tf::Quaternion quaternion(q.x, q.y, q.z, q.w);
	tf::Matrix3x3(quaternion).getRPY(r, p, y);
	return y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_navigator");

	TestNavigator test_navigator;
	test_navigator.process();

	ros::spin();
    return 0;
}
