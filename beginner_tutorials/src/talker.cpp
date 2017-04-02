#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <stdio.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

  ros::Rate loop_rate(10);
  system("stty raw");
  while (ros::ok())
  {
    char c = getchar();
    if (c == 'e'){
      ros::shutdown();
      system("stty cooked");
    }else{
      geometry_msgs::Twist twist;


	switch (c){
  	case 'w':
     		twist.linear.x = 3;
      		break;
    	case 's':
      		twist.linear.x = -3;
      		break;
    	case 'a':
      		twist.angular.z = 3;
      		break;
    	case 'd':
      		twist.angular.z = -3;
      		break;
    	default:
      		break;
  }


      chatter_pub.publish(twist);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  return 0;
}
