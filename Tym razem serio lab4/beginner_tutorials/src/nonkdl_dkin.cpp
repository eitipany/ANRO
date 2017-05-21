#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <math.h>



ros::Publisher pub;


void msgReceived(const sensor_msgs::JointState &jointState){
    double t1 = jointState.position[0];
    double t2 = jointState.position[1];
    double t3 = jointState.position[2];
    
	double a2  = 1.0;
	double d1 = 0.6;
	double rotation[4][4];
	rotation[0][0]=cos(t1)*cos(t2)*cos(t3)-cos(t1)*sin(t2)*sin(t3);
	rotation[1][0]=sin(t1)*cos(t2)*cos(t3)-sin(t1)*sin(t2)*sin(t3);
	rotation[2][0]=-1*cos(t2)*sin(t3)-sin(t2)*cos(t3);
	rotation[3][0]=0;
	rotation[0][1]=-1*cos(t1)*cos(t2)*sin(t3)-cos(t1)*sin(t2)*cos(t3);
	rotation[1][1]=-1*sin(t1)*cos(t2)*sin(t3)-sin(t1)*sin(t2)*cos(t3);
	rotation[2][1]=sin(t2)*sin(t3)-cos(t2)*cos(t3);
	rotation[3][1]=0;
	rotation[0][2]=-sin(t1);
	rotation[1][2]=cos(t1);
	rotation[2][2]=0;
	rotation[3][2]=0;
	rotation[0][3]=a2*cos(t1)*cos(t2);
	rotation[1][3]=a2*sin(t1)*cos(t2);
	rotation[2][3]=-1*a2*sin(t2)+d1;
	rotation[3][3]=1;


	

	double w =sqrt(1 + rotation[0][0] + rotation[1][1] + rotation[2][2])/2;
	double w4= 4*w;
	double x = (rotation[2][1] - rotation[1][2])/w4;
	double y = (rotation[0][2] - rotation[2][0])/w4;
	double z = (rotation[1][0] - rotation[0][1])/w4;
 
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id="base_link";
    
    poseStamped.header.stamp = ros::Time::now();
  

  	poseStamped.pose.position.x=rotation[0][3];
    	poseStamped.pose.position.y=rotation[1][3];
    	poseStamped.pose.position.z=rotation[2][3];


    
    poseStamped.pose.orientation.x=x;
    poseStamped.pose.orientation.y=y;
    poseStamped.pose.orientation.z=z;
    poseStamped.pose.orientation.w=w;
    pub.publish(poseStamped);

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "NONKDL_DKIN");
  ros::NodeHandle n;
  ros::Rate loop_rate(30);
  pub=n.advertise<geometry_msgs::PoseStamped>("nonkdl",1000);
  ros::Subscriber sub = n.subscribe("/joint_states", 100, msgReceived);
  
  ros::spin();

  return 0;
}
