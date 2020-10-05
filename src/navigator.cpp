#include "ros/ros.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

class Navigator{
public:
    Navigator();

    // callback
    void PoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void TargetCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    void process();

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    //subscriber
    ros::Subscriber pose_sub;
    ros::Subscriber target_sub;

    //publisher
    ros::Publisher local_goal_pub;

    float tolerance;
    bool finish;
    bool goal_subscribed;
    bool pose_subscribed;
    std::string ROBOT_FRAME;
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseWithCovarianceStamped current_pose;

    double normalize(double z);
    double angle_diff(double a, double b);
    double get_yaw(geometry_msgs::Quaternion q);
};

Navigator::Navigator()
    : private_nh("~")
{
    //subscriber
    pose_sub = nh.subscribe("/amcl_pose",1, &Navigator::PoseCallback, this);
    target_sub = nh.subscribe("/move_base_simple/goal", 1, &Navigator::TargetCallback, this);

    //publisher
    local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/target",1,true);

    ROBOT_FRAME = "base_link";
    tolerance = 0.3; //[m]
    goal_subscribed = false;
    pose_subscribed = false;
    finish = false;
}

void Navigator::TargetCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    goal = *msg;
    goal_subscribed = true;
    finish = false;
}

void Navigator::PoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    current_pose = *msg;
    pose_subscribed = true;
}

void Navigator::process()
{
    ros::Rate loop_rate(10);
    while(ros::ok()){
        geometry_msgs::PoseStamped local_goal;
        local_goal.header.stamp = ros::Time::now();
        local_goal.header.frame_id = ROBOT_FRAME;
        if(!finish && goal_subscribed && pose_subscribed){
            float dx = goal.pose.position.x - current_pose.pose.pose.position.x;
            float dy = goal.pose.position.y - current_pose.pose.pose.position.y;
            float dis = sqrt(dx*dx + dy*dy);
            if(dis < tolerance){
                std::cout << "goal!!!!!!!!" << std::endl;
                finish = true;
            }
            float yaw = angle_diff(atan2(dy,dx), get_yaw(current_pose.pose.pose.orientation));
            local_goal.pose.position.x = dis*cos(yaw);
            local_goal.pose.position.y = dis*sin(yaw);
            local_goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }else{
            std::cout << "========finish========" << std::endl;
            local_goal.pose.position.x = 0.0;
            local_goal.pose.position.y = 0.0;
            local_goal.pose.orientation.x = 0;
            local_goal.pose.orientation.y = 0;
            local_goal.pose.orientation.z = 0;
            local_goal.pose.orientation.w = 1;
        }
        local_goal_pub.publish(local_goal);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

double Navigator::normalize(double z)
{
    return atan2(sin(z),cos(z));
}

double Navigator::angle_diff(double a, double b)
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

double Navigator::get_yaw(geometry_msgs::Quaternion q)
{
    double r, p, y;
    tf::Quaternion quaternion(q.x, q.y, q.z, q.w);
    tf::Matrix3x3(quaternion).getRPY(r, p, y);
    return y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigator");

    Navigator navigator;
    navigator.process();

    return 0;
}
