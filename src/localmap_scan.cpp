/*
 *	localmap_scan.cpp
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>


/*global variables*/
nav_msgs::OccupancyGrid local_map;
sensor_msgs::LaserScan scan;
bool received_localmap = false;
double max_range = 10.0;

void index_to_point(nav_msgs::OccupancyGrid grid, int index, int& x, int& y)
{
	x = index%grid.info.width - grid.info.width/2.0;
	y = index/grid.info.width - grid.info.height/2.0;
	// std::cout << "index = " << index << std::endl;
	// std::cout << "x = " << x << std::endl;
	// std::cout << "y = " << y << std::endl;
}

int point_to_index(nav_msgs::OccupancyGrid grid, int x, int y)
{
	// std::cout << "- POINT TO INDEX -" << std::endl;
	int x_ = x + grid.info.width/2.0;
	int y_ = y + grid.info.height/2.0;
	return	y_*grid.info.width + x_;
}

void index_to_meterpoint(nav_msgs::OccupancyGrid grid, int index, double& x, double& y)
{
	x = (index%grid.info.width - grid.info.width/2.0 + 0.5)*grid.info.resolution;
	y = (index/grid.info.width - grid.info.height/2.0 + 0.5)*grid.info.resolution;
}

void callback_localmap(const nav_msgs::OccupancyGridConstPtr& msg)
{
	local_map = *msg;
	received_localmap = true;
}

double map_calc_range(nav_msgs::OccupancyGrid map, double ox, double oy, double oa)
{
	int x0,x1,y0,y1;
	int x,y;
	int xstep, ystep;
	int steep;
	int tmp;
	int deltax, deltay, error, deltaerr;

	x0 = (ox - map.info.origin.position.x) /map.info.resolution;
	y0 = (oy - map.info.origin.position.y) /map.info.resolution;

	x1 = (ox + max_range * cos(oa) -  map.info.origin.position.x) /map.info.resolution;
	y1 = (oy + max_range * sin(oa) -  map.info.origin.position.y) /map.info.resolution;
	
	if(abs(y1-y0) > abs(x1-x0))
		steep = 1;
	else
		steep = 0;
	if(steep){
		tmp = x0;
		x0 = y0;
		y0 = tmp;
		tmp = x1;
		x1 = y1;
		y1 = tmp;
	}

	deltax = abs(x1-x0);
	deltay = abs(y1-y0);
	error = 0;
	deltaerr = deltay;

	x = x0;
	y = y0;

	if(x0 < x1)
		xstep = 1;
	else
		xstep = -1;
	if(y0 < y1)
		ystep = 1;
	else
		ystep = -1;

	if(steep)
	{
		if(!((y >= 0) && (y < map.info.width) && (x >= 0) && (x < map.info.height))
				|| map.data[(y + x *map.info.width)] != 0)
			return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) *map.info.resolution;
	}
	else	
	{
		if(!((x >= 0) && (x < map.info.width) && (y >= 0) && (y < map.info.height))
				|| map.data[(x + y *map.info.width)] != 0)
			return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) *map.info.resolution;
	}
	while(x != (x1 + xstep * 1))
	{
		x += xstep;
		error += deltaerr;;
		if(2*error >= deltax)
		{
			y += ystep;
			error -= deltax;
		}

		if(steep)
		{
			if(!((y >= 0) && (y < map.info.width) && (x >= 0) && (x < map.info.height))
					|| map.data[(y + x *map.info.width)] != 0)
			return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map.info.resolution;
		}
		else
		{
			if(!((x >= 0) && (x < map.info.width) && (y >= 0) && (y < map.info.height))
					|| map.data[(x + y *map.info.width)] != 0)
			return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map.info.resolution;
		}
	}
	return max_range;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "localmap_scan");
	ros::NodeHandle nh;

	/*sub*/
	ros::Subscriber sub_grid_lidar = nh.subscribe("/local_map", 1, callback_localmap);
	
	/*pub*/
	ros::Publisher pub_scan = nh.advertise<sensor_msgs::LaserScan>("/local_map/scan",1);
	
	/*tf*/
	tf::TransformListener listener;
	tf::StampedTransform transform;
	
	/*variables*/
	sensor_msgs::LaserScan scan;
	
	/*initialization*/
	const float num_scan_data = 720.0;
	scan.header.frame_id = "base_link";
	scan.angle_min = -M_PI / 2.0;
	scan.angle_max =  M_PI / 2.0;
	scan.angle_increment = M_PI / num_scan_data;
	scan.scan_time = 0.025;
	scan.time_increment = scan.scan_time / (num_scan_data * 2.0);
	scan.range_min = 0.10;
	scan.range_max = 10.0;
	scan.ranges.resize(int(num_scan_data));

	/*loop*/
	ros::Rate loop_rate(10);

	while(ros::ok()){
		if(received_localmap && !local_map.data.empty()){
			try{
				listener.waitForTransform(local_map.header.frame_id,"/base_link",local_map.header.stamp,ros::Duration(1.0));
				listener.lookupTransform(local_map.header.frame_id, "/base_link",local_map.header.stamp,transform);
			}
			catch(tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}
			double roll,pitch,yaw;
			tf::Matrix3x3(transform.getRotation()).getRPY(roll,pitch,yaw);

			for(int i = 0; i<num_scan_data; i++){
				float angle = yaw + i*scan.angle_increment + scan.angle_min;
				scan.ranges[i] = map_calc_range(local_map,0,0,angle);
				scan.header.stamp = ros::Time::now();
			}
			pub_scan.publish(scan);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}
