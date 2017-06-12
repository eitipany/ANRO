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
 double a2  = 1.0;
                        double d1 = 0.6;
                        double a3 = 1.0;

void sigintHandler(int) {
	exitFlag = 1;
}
 
//publishTrajectoryPath
geometry_msgs::PoseStamped poseStamped; 
int main(int argc, char **argv){
	


	



	ROS_INFO("tu je "); 	
	double x,y,z,t, xxx, yyy, zzz;
	double ox, oy, oz; 
	double dx, dy, dz; 
	
	ros::init(argc, argv, "jint");
	ros::NodeHandle s; 
	ros::NodeHandle n; 
	ros::Rate loop_rate(10);
	ros::Publisher pub = s.advertise<geometry_msgs::PoseStamped>("jint",1);
		ros::Publisher ukl = s.advertise<geometry_msgs::PoseStamped>("ukl",1000);
	ros::Publisher joint = s.advertise<sensor_msgs::JointState>("joint_states",1);  
	ros::Publisher path_pub;
        path_pub=n.advertise<nav_msgs::Path>("path1",1);
        nav_msgs::Path path;
        std::vector<geometry_msgs::PoseStamped> plan;

	
	if(!n.getParam("a2", a2))
		{
			ROS_ERROR("Błąd pobrania. Użyte domyślne wartości.");	
		}
	if(!n.getParam("a3", a3))
		{
			ROS_ERROR("Błąd pobrania. Użyte domyślne wartości.");	
		}

	if(!n.getParam("d1", d1))
		{
			ROS_ERROR("Błąd pobrania. Użyte domyślne wartości.");	
}

//double d1,d2,d3;

//	s.param<double>("d1",d1,3);
 //   	s.param<double>("d2",d2,3);
//	s.param<double>("d3",d3,3);

	sensor_msgs::JointState robotState; 
	robotState.name.push_back("part0part1");
	robotState.name.push_back("part1part2");
	robotState.name.push_back("part2part3");
	robotState.position.resize(robotState.name.size());
	

//	d1 = 3;
//	d2 = 0;
//	d3 = 1.5;
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

		
		ROS_INFO("Podaj zadane x y z i czas ");  
		std::cin>>xxx>>yyy>>zzz>>t;
		int init=1;
		
		if (t==0){
		ROS_INFO("kwadratto ");
		
		for (double ii = 0; ii<= 4.1; ii +=0.1){
			
			if (init==1){
			t=1;
			xxx=1;
			yyy=1;
			zzz=0;
			init=0;
			
			}

			else if (ii <=1){
				
				xxx=1;
				yyy=1;
				zzz=ii;
				t=0.1;
			}
			else if (ii <= 2){
				xxx=2-ii;
				yyy=1;
				zzz=1;
				t=0.1;
			}
			else if (ii <= 3){
				xxx=0;
				yyy=1;
				zzz=3-ii;
				t=0.1;
			}
			else if (ii <=4.1){
				xxx=ii-3;
				yyy=1;
				zzz=0;
				t=0.1;
			}

			double e,f,c,A,B,G,cosB,cosG;
        
    double eps=0.01;
    
    double a22=pow(a2,2);
    double a32=pow(a3,2);
    
    double x0=0;
    double y0=d1;
    double x1=sqrt(pow(xxx,2)+pow(yyy,2));
    double y1=d1;
    double x2=x1;
    double y2=zzz;
    
    c=sqrt(pow(x2-x0,2)+pow(y2-y0,2));
    e=sqrt(pow(x1-x0,2)+pow(y1-y0,2));
    f=y2-y1;
    
    A=atan(f/e);
    
    cosB=(a32-a22-pow(c,2))/(-2*a2*c);
    B=acos(cosB);
    
    cosG=(pow(c,2)-a22-a32)/(-2*a2*a3);
    G=acos(cosG);
           
	x=atan2(yyy,xxx);
	y=-A-B;
	z=M_PI-G;
	
	

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
			
			
	 		KDL::Chain chain;

        		chain.addSegment(Segment(Joint(Joint::None),Frame(Frame::DH(0, 0, 0, 0))));
        		chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(0, 0, d1, 0))));
        		chain.addSegment(Segment(Joint(Joint::None),Frame(Frame::DH(0, -M_PI/2, 0, 0))));
        		chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(a2, 0, 0, 0))));
        		chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(a3, 0, 0, 0))));
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
			ukl.publish(poseStamped); 
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
			
		}

		else if (t!=0){
		
		//zzz=zzz+0.6;
		double e,f,c,A,B,G,cosB,cosG;
        
    double eps=0.01;
    
    double a22=pow(a2,2);
    double a32=pow(a3,2);
    
    double x0=0;
    double y0=d1;
    double x1=sqrt(pow(xxx,2)+pow(yyy,2));
    double y1=d1;
    double x2=x1;
    double y2=zzz;
    
    c=sqrt(pow(x2-x0,2)+pow(y2-y0,2));
    e=sqrt(pow(x1-x0,2)+pow(y1-y0,2));
    f=y2-y1;
    
    A=atan(f/e);
    
    cosB=(a32-a22-pow(c,2))/(-2*a2*c);
    B=acos(cosB);
    
    cosG=(pow(c,2)-a22-a32)/(-2*a2*a3);
    G=acos(cosG);
           
	x=atan2(yyy,xxx);
	y=-A-B;
	z=M_PI-G;
	
	

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
			
			
	 		KDL::Chain chain;

        		chain.addSegment(Segment(Joint(Joint::None),Frame(Frame::DH(0, 0, 0, 0))));
        		chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(0, 0, d1, 0))));
        		chain.addSegment(Segment(Joint(Joint::None),Frame(Frame::DH(0, -M_PI/2, 0, 0))));
        		chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(a2, 0, 0, 0))));
        		chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(a3, 0, 0, 0))));
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
			ukl.publish(poseStamped); 
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
	}
	ros::spin();

	return 0;
}
