/*
This is the Request node. User will request to make modifications with respect to the
speed of the robot. Also the user can request to reset the position of the robot.
*/
/*
The first section is to include all important headers for the node.
*/
#include "ros/ros.h" // Required node for ros.
#include "geometry_msgs/Twist.h" /* This is used to express velocity in free space
normally broken down into linear and angular parts.*/
#include "sensor_msgs/LaserScan.h" /* This is used to single scan from a planar laser
range-finder.*/
#include "second_assignment/robotspeed.h" /* This is used to get messages with regards to
the input command and the resultant speed. */

#include "std_srvs/Empty.h" /* This header is used when no actual data is exchanged
between the service and the client. This has been used in order to invoke a reset command
in the order to put the robot back in its initial position.*/

// Using a service client to change speed of the robot.
ros::ServiceClient robot_speed;

//std_srvs::Empty reset;
second_assignment::robotspeed robspeed;


// In this node, I have included only the function that allows the user to input the
//character they want in order to manipulate the speed of the robot.

// Main function to get input from the User.
int main(int argc, char **argv){

ros::init(argc, argv, "userinput_node");
ros::NodeHandle n;

/*
 In this assignment we are using a Server-Client since it tends to be more request
 response. In Server-Client, a client makes an explicit request for some specific data.
 Publisher-Subscriber tends to be for different purposes where you subscribe to
 something, process it and publish it. Usually at a fairly fixed rate. You can subscribe
 to multiple things and publish multiple things. Multiple other nodes can subscribe to
 the same topic. You can even have multiple nodes publishing the same topic. 
*/
robot_speed = n.serviceClient<second_assignment::robotspeed>("manipulate_speed");

/* After the node is initiated, we need to set a set a rate at which the node is working.
The rate is in Hz. ros::Rate is a class provided by roslib which akes a best effort at
maintaining a particular rate for a loop . Since the rate is in Hz, we have to 
keep in mind the time (in seconds) and then basically follow the formula f=1/t. Based on
ROS documentations and examples, I will set it to 10Hz which means that t=0.1 seconds.  
*/
ros::Rate r(10);

char ch;

while(ros::ok())
{
 // This section would be the output giving instructions to the user.
 std::cout << "Below are the instructions you need to make the robot move\n";
 std::cout << "Press w to increase the speed\n";
 std::cout << "Press s to decrease the speed\n";
 std::cout << "Press r to reset the robot\n";
 std::cout << "Press any other button to get out!\n";
 std::cin >> ch;
 
 /*
 Steps to ensure:
 1. We first need to make sure that the service is ready before calling it.
 2. When the user presses the button, there should be a request sent to either increase,
 decrease or reset the position of the robot.
 3. After requesting, the custom service we created is called in the case where the speed
 needs to increase or decrease, whereas when the reset is requested an Empty Service is called. For an Empty service, no actual data is exchanged between the service and the client.
 */
 
 switch(ch){
 case ('w'):
     // This will request the robot to move faster.
     robot_speed.waitForExistence();
     robspeed.request.command = 'w';
     robot_speed.call(robspeed);
     std::cout<<"You have chosen to increase the speed. Increasing speed!!\n";
 break;
 case ('s'):
     // This will request the robot to move slower.
     robot_speed.waitForExistence();
     robspeed.request.command = 's';
     robot_speed.call(robspeed);
     std::cout<<"You have chosen to decrease the speed. Decreasing speed!!\n";
 break;
 case ('r'):
     // This will request the robot to reset its position.
     robot_speed.waitForExistence();
     robspeed.request.command = 'r';
     robot_speed.call(robspeed);
     std::cout<<"Resetting the position of the robot!!\n";
 break;
 default:
     // In case some other button is pressed.
     // In this case ros::shutdown is used kill the node.
     std::cout<<"Wrong Command! Shutting down User Control! Goodbye!";
     ros::shutdown();
 break;        
 }
// Here the ROS Rate will sleep for the amount of time needed to complete the 0.1 seconds.
// If the code takes longer, the sleep will be shorter.
r.sleep(); 
}
// Ros spin is used in order to create a loop for the user to keep inputting a command.
ros::spin();
return 0;
}
