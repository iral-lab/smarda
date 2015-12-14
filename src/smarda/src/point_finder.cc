/*
 * point_finder.cc : C++ code to take a point cloud image and
 *                   find cylinders.
 * 
 */

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>


#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/pcd_io.h>


enum smarda_colors {
	RED = 0,
	GREEN,
	BLUE
};

/* global state */
ros::NodeHandle *nh;
ros::Subscriber point_cloud;		/* Global kinect subscriber */
enum smarda_colors current_color;
ros::Publisher location;

static void find_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
				int delta, float *x, float *y, float *z)
{
	int maxr = 0, maxg = 0, maxb = 0;
	int avgr = 0, avgg = 0, avgb = 0;
	float maxx = 0, maxy = 0, maxz = 0;
	float avgx = 0, avgy = 0, avgz = 0;
	int pts = 0;

	switch(current_color) {
		case RED:
			BOOST_FOREACH(pcl::PointXYZRGB pt, cloud->points) {
				if (pt.r > pt.b+delta && pt.r > pt.g+delta) {
					if (pt.r > maxr) 
						maxr = pt.r;
					avgr += pt.r;
					if (pt.g > maxg) 
						maxg = pt.g;
					avgg += pt.g;
					if (pt.b > maxb) 
						maxb = pt.b;
					avgb += pt.b;
					if (pt.x > maxx) 
						maxx = pt.x;
					avgx += pt.x;
					if (pt.y > maxy) 
						maxy = pt.y;
					avgy += pt.y;
					if (pt.z > maxz) 
						maxr = pt.z;
					avgz += pt.z;
					pts += 1;
				}
			
			}
			break;

		case GREEN:
			BOOST_FOREACH(pcl::PointXYZRGB pt, cloud->points) {
				if (pt.g > pt.b+delta && pt.g > pt.r+delta) {
					if (pt.r > maxr) 
						maxr = pt.r;
					avgr += pt.r;
					if (pt.g > maxg) 
						maxg = pt.g;
					avgg += pt.g;
					if (pt.b > maxb) 
						maxb = pt.b;
					avgb += pt.b;
					if (pt.x > maxx) 
						maxx = pt.x;
					avgx += pt.x;
					if (pt.y > maxy) 
						maxy = pt.y;
					avgy += pt.y;
					if (pt.z > maxz) 
						maxr = pt.z;
					avgz += pt.z;
					pts += 1;
				}
			
			}
			break;

		case BLUE:
			BOOST_FOREACH(pcl::PointXYZRGB pt, cloud->points) {
				if (pt.b > pt.r+delta && pt.b > pt.g+delta) {
					if (pt.r > maxr) 
						maxr = pt.r;
					avgr += pt.r;
					if (pt.g > maxg) 
						maxg = pt.g;
					avgg += pt.g;
					if (pt.b > maxb) 
						maxb = pt.b;
					avgb += pt.b;
					if (pt.x > maxx) 
						maxx = pt.x;
					avgx += pt.x;
					if (pt.y > maxy) 
						maxy = pt.y;
					avgy += pt.y;
					if (pt.z > maxz) 
						maxr = pt.z;
					avgz += pt.z;
					pts += 1;
				}
			
			}
			break;
	}
	if (pts < 10) {
		*x = 0.0;
		*y = 0.0;
		*z = 0.0;
		ROS_INFO("There was no object detected of that color");
		return;
	} 



	avgr /= pts;
	avgg /= pts;
	avgb /= pts;
	avgx /=	pts;
	avgy /= pts;
	avgz /= pts; 

	ROS_INFO("There's something %d at (%d pts) at approx (%f, %f, %f) (avg color %d, %d, %d)",
						current_color, pts, avgx, avgy, avgz, avgr, avgg, avgb);

	*x = avgx;
	*y = avgy;
	*z = avgz;
	return;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
				new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(*msg, *cloud);
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
				new pcl::PointCloud<pcl::PointXYZRGB>);

	ROS_INFO("Got cloud message\n");

	/* Filter out Z ranges beyond 0.9 meters */
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.2, 1.5);
	pass.filter (*cloud_filtered);

	ROS_INFO("PointCloud after filtering has: %zd data points", 
						cloud_filtered->points.size());

	float x,y,z;

	find_color(cloud_filtered, 50, &x, &y, &z);

	// XXXjc: Transform
	/* 
	 * sqrt(z^2 + y^2 - height^2) = y_prime
	 * delta_x = .56
	 * delta_y = .975
	 * height = .215
	 * new x = x -delta_x
         * new y = y - delta_y;
         */
	ROS_INFO("Transforming");
	float delta_x = 0.66;    /* Measurements */
	float delta_y = 1.045;   /* Measurements */
	float x_prime, y_prime;
	x_prime = -delta_x - x;
	y_prime = delta_y - sqrt((z*z)+(y*y)+(0.215*0.215));
	ROS_INFO("Believe translated coordinates are (%f, %f)", x_prime, y_prime);
	std_msgs::String locmsg;

        if (x == 0.0 && y == 0.0 && z == 0.0) {
		x_prime = 0.0;
		y_prime = 0.0;
	}

	std::stringstream ss;
	ss << x_prime << " " << y_prime ;
	locmsg.data = ss.str();
	ROS_INFO("Sending %s", locmsg.data.c_str());
	
	location.publish(locmsg);

	point_cloud.shutdown();

	return;

}

void color_cb(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I'm told I should look for a %s bottle", msg->data.c_str());

	/* Set the global that specifies the color we're looking for */
	if(msg->data == "blue") {
		current_color = BLUE;
	} else if (msg->data == "red") {
		current_color = RED;
	} else if (msg->data == "green") {
		current_color = GREEN;
	} else {
		ROS_INFO("invalid color: %s", msg->data.c_str());
		return;
	}

	/* Subscribe to the point cloud messages now */
	point_cloud = nh->subscribe("/kinect2/hd/points", 1, cloud_cb);
	
	return;
}

int main(int argc, char** argv)
{
	// Initialize
	ros::init(argc, argv, "smarda");
	nh = new ros::NodeHandle();

	ROS_INFO("Listening");

	ros::Subscriber color_listener = nh->subscribe("/smarda/bottle_color", 10, color_cb);
	location = nh->advertise<std_msgs::String>("/smarda/location", 2); 	

	// Spin
	ros::spin();
}
