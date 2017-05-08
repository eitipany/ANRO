
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

using namespace KDL;

ros::Publisher pub;

void msgReceived(const sensor_msgs::JointState &jointState){
	double t1 = jointState.position[0];
	double t2 = jointState.position[1];
        double t3 = jointState.position[2];
	
	double a2  = 1.0;
        double d1 = 0.6;


	 KDL::Chain chain; // Stworzenie łańcucha kinematycznego

        chain.addSegment(Segment(Joint(Joint::None),Frame(Frame::DH(0, 0, 0, 0))));
        chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(0, 0, d1, 0))));
        chain.addSegment(Segment(Joint(Joint::None),Frame(Frame::DH(0, -M_PI/2, 0, 0))));
        chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(a2, 0, 0, 0))));
        chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(0, 0, 0, 0))));

        ChainFkSolverPos_recursive solver(chain);
        JntArray q(chain.getNrOfJoints());
        Frame F;
	
	// while(ros::ok()) {
         //       ros::spinOnce(); // Pobranie informacji od Joint_State_Publishera


                q(0)=t1;
                q(1)=t2;
                q(2)=t3;

                solver.JntToCart(q,F);

                double x,y,z=0; // Współrzędne końcówki

                x = F.p.data[0];
                y = F.p.data[1]; 
                z = F.p.data[2];

                q(0)=t1/2;
                q(1)=t2/2;
                q(2)=t3/2;
                solver.JntToCart(q,F);





	geometry_msgs::PoseStamped poseStamped;
    	poseStamped.header.frame_id="base_link";
    
	poseStamped.header.stamp = ros::Time::now();
  
	poseStamped.pose.position.x=x;
        poseStamped.pose.position.y=y;
        poseStamped.pose.position.z=z;

	 poseStamped.pose.orientation.w = F.M.data[0];
         poseStamped.pose.orientation.x = F.M.data[4];
         poseStamped.pose.orientation.y = -F.M.data[1];
         poseStamped.pose.orientation.z = F.M.data[3];
	pub.publish(poseStamped);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "KDL_DKIN");
  ros::NodeHandle n;
  ros::Rate loop_rate(30);
  pub=n.advertise<geometry_msgs::PoseStamped>("kdl",1000);
  ros::Subscriber sub = n.subscribe("/joint_states", 100, msgReceived);
  
  ros::spin();

  return 0;
}

