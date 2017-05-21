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
 
//publishTrajectoryPath
geometry_msgs::PoseStamped poseStamped; 
int main(int argc, char **argv){
	ROS_INFO("tu je "); 	
	double x,y,z,t;
	double ox, oy, oz; 
	double dx, dy, dz; 
	
	ros::init(argc, argv, "jint");
	ros::NodeHandle s; 
	ros::NodeHandle n; 
	ros::Rate loop_rate(10);
	ros::Publisher pub = s.advertise<geometry_msgs::PoseStamped>("jint",1);
	ros::Publisher joint = s.advertise<sensor_msgs::JointState>("joint_states",1);  
	ros::Publisher path_pub;
        path_pub=n.advertise<nav_msgs::Path>("path1",1);
        nav_msgs::Path path;
        std::vector<geometry_msgs::PoseStamped> plan;

	double d1,d2,d3;

//	s.param<double>("d1",d1,3);
 //   	s.param<double>("d2",d2,3);
//	s.param<double>("d3",d3,3);

	sensor_msgs::JointState robotState; 
	robotState.name.push_back("part0part1");
	robotState.name.push_back("part1part2");
	robotState.name.push_back("part2part3");
	robotState.position.resize(robotState.name.size());
	

	d1 = 3;
	d2 = 0;
	d3 = 1.5;
	ox=0; 
	oy=0; 
	oz=0; 

	robotState.header.stamp = ros::Time::now(); 	
	robotState.position[0] = ox;
	robotState.position[1] = oy;
	robotState.position[2] = oz;
	joint.publish(robotState); 
	ROS_INFO("tu tez je "); 
	while(ros::ok()|| !exitFlag){

		
		ROS_INFO("Podaj zadane katy na 1 2 3 joincie oraz czas : alfa1 alfa2 alfa3 t ");  
		std::cin>>x>>y>>z>>t;
		if(x>d1 || x<0 || y>0 || y<-3 || z<-1.5 || z>1.5 || t<0)
		{
			ROS_INFO("niepoprawne parametry");
			continue;
		}
		else
			ROS_INFO("zadane wartosci: alfa1=%f, alfa2=%f, alfa3=%f, t=%f",x,y,z,t); 


		dx = (x-ox)/(10*t); 
		dy = (y-oy)/(10*t); 
		dz = (z-oz)/(10*t); 

		for(int i=0; i<(int)10*t; i++){
			robotState.header.stamp = ros::Time::now(); 
		
			robotState.position[0] +=dx;
			robotState.position[1] +=dy;
			robotState.position[2] +=dz;
			double t1 = robotState.position[0];
    			double t2 = robotState.position[1];
			double t3 = robotState.position[2];
			
			double a2  = 1.0;
        		double d1 = 0.6;


	 		KDL::Chain chain;

        		chain.addSegment(Segment(Joint(Joint::None),Frame(Frame::DH(0, 0, 0, 0))));
        		chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(0, 0, d1, 0))));
        		chain.addSegment(Segment(Joint(Joint::None),Frame(Frame::DH(0, -M_PI/2, 0, 0))));
        		chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(a2, 0, 0, 0))));
        		chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(0, 0, 0, 0))));

        		ChainFkSolverPos_recursive solver(chain);
        		JntArray q(chain.getNrOfJoints());
			Frame F;
			
			  q(0)=t1;
                	  q(1)=t2;
                	  q(2)=t3;

                		solver.JntToCart(q,F);

                		double xx,yy,zz=0;

                		xx = F.p.data[0];
                		yy = F.p.data[1]; 
                		zz = F.p.data[2];

                		q(0)=t1/2;
                		q(1)=t2/2;
                		q(2)=t3/2;
				solver.JntToCart(q,F);

			ROS_INFO("aktualne : x=%f, y=%f, z=%f",xx,yy,zz); 			
			poseStamped.header.stamp = ros::Time::now();
                        poseStamped.header.frame_id="base_link";
                        poseStamped.pose.position.x=xx;
                        poseStamped.pose.position.y=yy;
                        poseStamped.pose.position.z=zz; 			
			
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


                        //pub.publish(poseStamped); 
                        path_pub.publish(path);


			joint.publish(robotState); 
			loop_rate.sleep();
		}
		
		ox = robotState.position[0]; 
		oy = robotState.position[1];
		oz = robotState.position[2];

		loop_rate.sleep();
	}
	
	ros::spin();

	return 0;
}
