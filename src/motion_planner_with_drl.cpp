#include "ros/ros.h"
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <torch/script.h>

class MPDRL{
	public:
		MPDRL();
		void ScanCallback(const sensor_msgs::LaserScanConstPtr& msg);
		void TargetCallback(const geometry_msgs::PoseStampedConstPtr& msg);

		void process();

	private:
		ros::NodeHandle nh;
		ros::NodeHandle private_nh;

		// Subscriber
		ros::Subscriber scan_sub;
		ros::Subscriber target_sub;

		// Publisher
		ros::Publisher vel_pub;

		torch::jit::script::Module module;
		geometry_msgs::Twist vel;
		bool target_received;
		bool scan_received;
		float dis;
		float yaw;
		double pre_v;
		double pre_w;
		double epsilon;
		float obs[36];
		int HZ;
		double MAX_SPEED;
		double MAX_YAWRATE;
		std::string MODEL_PATH;

		double get_yaw(geometry_msgs::Quaternion q);
};

MPDRL::MPDRL()
	:private_nh("~")
{
	// Subscriber
	scan_sub = nh.subscribe("/local_map/scan", 1, &MPDRL::ScanCallback, this);
	target_sub = nh.subscribe("/local_goal", 1, &MPDRL::TargetCallback, this);

	// Publisher
	vel_pub = nh.advertise<geometry_msgs::Twist>("/local_path/cmd_vel", 1, true);

	private_nh.param("HZ", HZ, {20});
	private_nh.param("EPSILON", epsilon, {1.0});
	private_nh.param("MAX_SPEED", MAX_SPEED, {1.0});
	private_nh.param("MAX_YAWRATE", MAX_YAWRATE, {1.0});
	private_nh.param("MODEL_PATH", MODEL_PATH, {"model.pt"});
	std::cout << "Model path : " << MODEL_PATH << std::endl;

	module = torch::jit::load(MODEL_PATH);
	target_received = false;
	scan_received = false;
	dis = 0.0;
	yaw = 0.0;
	pre_v = 0.0; 
	pre_w = 0.0; 
}

void MPDRL::ScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
	sensor_msgs::LaserScan scan;
	scan = *msg;
	for(int i=0; i<sizeof(obs)/sizeof(obs[0]); i++){
		float min_range = scan.range_max;
		for(int j=0; j<20; j++){
			float r = scan.ranges[i*20+j];
			if(r < min_range){
				min_range = r;
			}
		}
		obs[i] = min_range;
	}
	scan_received = true;
}

void MPDRL::TargetCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	geometry_msgs::PoseStamped local_goal;
	local_goal = *msg;
	yaw = get_yaw(local_goal.pose.orientation);
	float x = local_goal.pose.position.x;
	float y = local_goal.pose.position.y;
	dis = sqrt(x*x + y*y);
	target_received = true;
}

void MPDRL::process()
{
	ros::Rate loop_rate(HZ);
	while(ros::ok()){
		std::cout << "=======================" << std::endl;
		std::cout << "motion_planner_with_drl" << std::endl;
		double v = 0.0;
		double w = 0.0;
		if(target_received && scan_received){
			std::vector<float> state;
			for(auto o : obs){
				state.push_back(o);
			}
			state.push_back(dis);
			state.push_back(sin(yaw));
			state.push_back(cos(yaw));
			torch::Tensor input = torch::tensor({state}).view({1,39});
			auto output = module.forward({input}).toTensor();
			v = (output[0][0].item<float>() + 1.0) * 0.5 * MAX_SPEED;
			w = -MAX_YAWRATE + (output[0][1].item<float>() + 1.0) * MAX_YAWRATE;
			v = std::min(std::max(0.0, v), MAX_SPEED);
			w = std::min(std::max(-MAX_YAWRATE, w), MAX_YAWRATE);
		}else{
			std::cout << "local goal : " << target_received << std::endl;
			std::cout << "scan       : " << scan_received << std::endl;
		}
		v = std::min(std::max(pre_v-epsilon, v), pre_v+epsilon);
		w = std::min(std::max(pre_w-epsilon, w), pre_w+epsilon);
		vel.linear.x = v;
		vel.angular.z = w;
		std::cout <<  vel << std::endl;
		vel_pub.publish(vel);
		pre_v = v;
		pre_w = w;
		loop_rate.sleep();
		ros::spinOnce();
	}
}

double MPDRL::get_yaw(geometry_msgs::Quaternion q)
{
	double r, p, y;
	tf::Quaternion quaternion(q.x, q.y, q.z, q.w);
	tf::Matrix3x3(quaternion).getRPY(r, p, y);
	return y;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motion_planner_with_drl");

	MPDRL mpdrl;
	mpdrl.process();
	
	return 0;
}
