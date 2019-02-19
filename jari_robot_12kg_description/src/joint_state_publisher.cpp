  #include "ros/ros.h"
  #include "std_msgs/Header.h"
  #include "sensor_msgs/JointState.h"
  #include <sstream>

  int main(int argc, char **argv)
  {
     ros::init(argc, argv, "joint_states");
     ros::NodeHandle n;
     ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_state_publisher", 10);
     ros::Rate loop_rate(500);
     std_msgs::Header header;
     sensor_msgs::JointState joint_state;
     while (ros::ok())
     {
       /**
        * This is a message object. You stuff it with data, and then publish it.
        */
       
       ROS_INFO("%s", msg.data.c_str());
   
      pub.publish(joint_state); 
      ros::spinOnce();
  
      loop_rate.sleep();
    }
    return 0;
}
