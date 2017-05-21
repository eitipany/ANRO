#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <csignal>
#include <nav_msgs/Path.h>
using namespace std;
using namespace KDL;

volatile int exitFlag = 0;

void sigintHandler(int) {
	exitFlag = 1;
}
geometry_msgs::PoseStamped poseStamped; 

//publishTrajectoryPath

int main(int argc, char **argv){
	
	double x,y,z,t;
	double ox, oy, oz, oxx, oyy, ozz, oww; 
        double dx, dy, dz, d1, d2, d3; 	
	double dxx, dyy, dzz, dww, xx, zz, yy, ww;	
        ros::init(argc, argv, "oint");
	ros::NodeHandle s; 
	ros::NodeHandle n; 

	ros::Rate loop_rate(10);
	ros::Publisher pub = s.advertise<geometry_msgs::PoseStamped>("pose",1000);
	ros::Publisher path_pub;
	path_pub=n.advertise<nav_msgs::Path>("path",1);
	nav_msgs::Path path;
	std::vector<geometry_msgs::PoseStamped> plan;


	d1 = 3;
	d2 = 3;
	d3 = 3;
	ox=0; 
	oy=0; 
	oz=0; 
	oxx=0;
	oyy=0;
	ozz=0;
	oww=0;

	poseStamped.header.stamp = ros::Time::now();

	poseStamped.pose.position.x=ox;
        poseStamped.pose.position.y=oy;
        poseStamped.pose.position.z=oz; 	
	
	poseStamped.pose.orientation.w = oww;                        
	poseStamped.pose.orientation.x = oxx;
        poseStamped.pose.orientation.y = oyy;
        poseStamped.pose.orientation.z = ozz;

	
	while(ros::ok()|| !exitFlag){

		
		ROS_INFO("Podaj zadana pozycje x y z , kwaterniony x y z w oraz czas ruchu t");  
		std::cin>>x>>y>>z>>xx>>yy>>zz>>ww>>t;
		if(t<0)
		{
			ROS_INFO("czas musi byc >0");
			continue;
		}
		else
			ROS_INFO("Zadana pozycja : x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f,  t=%f",x,y,z, xx, yy, zz, ww, t); 


		dx = (x-ox)/(10*t); 
		dy = (y-oy)/(10*t); 
		dz = (z-oz)/(10*t); 

		dxx = (xx-oxx)/(10*t); 
                dyy = (yy-oyy)/(10*t); 
                dzz = (zz-ozz)/(10*t);
		dww = (ww-oww)/(10*t);  
		
		//std::vector<geometry_msgs::PoseStamped> plan;

		for(int i=0; i<(int)10*t; i++){
			poseStamped.header.stamp = ros::Time::now();
  			poseStamped.header.frame_id="base_link";
 		 	poseStamped.pose.position.x+=dx;
    			poseStamped.pose.position.y+=dy;
			poseStamped.pose.position.z+=dz; 
			poseStamped.pose.orientation.w += dww;
			poseStamped.pose.orientation.x += dxx;
			poseStamped.pose.orientation.y += dyy;
			poseStamped.pose.orientation.z += dzz;
			plan.push_back(poseStamped);
			poseStamped.header.stamp = ros::Time::now();
  			//int j = path.size();
			path.poses.resize(plan.size());

			if(!plan.empty()){
      				path.header.frame_id = "base_link";
      				path.header.stamp = plan[0].header.stamp;
			}

			for(int j=0; j < plan.size(); j++){
      				path.poses[j] = plan[j];
			}
  			
			
			pub.publish(poseStamped); 
			path_pub.publish(path);

			loop_rate.sleep();
		}
		
		ox = poseStamped.pose.position.x ; 
		oy = poseStamped.pose.position.y;
		oz = poseStamped.pose.position.z;
		oww = poseStamped.pose.orientation.w;
                oxx = poseStamped.pose.orientation.x;
                oyy = poseStamped.pose.orientation.y;
                ozz = poseStamped.pose.orientation.z;

		loop_rate.sleep();
	}
	
	ros::spin();

	return 0;
}
