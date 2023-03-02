/*
The first section is to include all important headers for the node.
*/
#include "ros/ros.h" // Required node for ros.
#include "geometry_msgs/Twist.h" /* This is used to express velocity in free space
normally broken down into linear and angular parts.*/
#include "sensor_msgs/LaserScan.h" /* This is used to single scan from a planar laser
range-finder.*/
#include "second_assignment/robotspeed.h" /* This is used to get messages with regards to
the input command and the resultant speed.*/
#include "std_srvs/Empty.h"

/* As one of the notes mentioned in the assignment is that the robot may crash if the
speed is increased too much. So we will set a minimum speed and a maximum speed. Since
the response to the speed is a float value in the service message, it would only make
sense to make the min and max speeds as float values as well. */
float min_sp = 0.0;
float max_sp = 3.0; // Subject to change after testing.
// Initializing the variable that will change the resultant speed based on the input.
float manip_sp = 0.0;

// Here I have also defined threshold for the control of the linear distance just like I
// did in the first assignment.
float d_th = 3.0;
// For any for-loop
float laser_scanning[720];
int i;

// Publisher is set that is global throughout
ros::Publisher pub;
// A Service Server created to deal with the change in speed
ros::ServiceServer ch_in_sp;
// Empty Service
std_srvs::Empty reset;
// Setting the initial velocity of the robot to 0
float velocity = 0.0;



/*
Here we are call the following functions:
1. A function to get the respond to the request put forward by the user.
2. A function that will calculate the minimum distance of the robot from an obstacle.
   The function is also used to avoid any obstacle on either the left,
right or front of the robot. If it isn't then it can move forward.
*/



// 1. Function to respond to the user input
bool speed_response(second_assignment::robotspeed:: Request& req,
                    second_assignment::robotspeed:: Response& res )
{
switch(req.command){
case('w'):

    // Will put an increment of 0.2 everytime user asks to increase speed to the point
    // that the speed reaches the maximum.
    manip_sp += 0.2;
    if(manip_sp>max_sp){
    std::cout<<"You have already reached the max speed. I would advice you to decrease the speed around corners as the robot might crash with the boundary.\n";
    manip_sp=max_sp;
    }
break;

case('s'):

    // Will put an decrement of 0.2 everytime user asks to decrease speed to the point
    // that the speed reaches the minimum.
    manip_sp -= 0.2;
    if(manip_sp<min_sp){
    std::cout<<"Seems like you have come to a hault! Please increase the speed of the robot to continue the race!\n";
    manip_sp=min_sp;
    }
break;

case('r'):
     // This will request the robot to reset its position.
     ros::service::call("/reset_positions", reset);
     req.command = 't';
break;     

default:
     // In case some other button is pressed.
     // In this case ros::shutdown is used kill the node.
     std::cout<<"Wrong Command! Goodbye!";
     ros::shutdown();
break;
}    
// A response to the request needs to be returned.
res.rspeed = manip_sp;
// A boolean function would have to return either a true or a false.
return true;
}


// 2. Function to calculate the minimum distance of the robot from the obstacle.
// The callback function that processes information from the Laser.
/* A hint was given to divide the ranges vector into subsections, and take the minimum
of each subsection. This will provide information about the closest obstacles. 
There are a total of 721 elements. So I am going to divide then from 0 to 100, 300 to 400
and 621 to 721.
*/

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scanning){
// Here I will be using geometry_msgs:: Twist as it is used to express velocity in free space normally broken down into linear and angular parts.
geometry_msgs::Twist vel;

// The above for loop will help us with the function that calculates the minimum
// distance.

// For Front side of the robot
float front = 0;
// For Right side of the robot
float right = 0;
// For Left side of the robot
float left = 0;

/*
Based on the file my_world.world, the following information was achieved with respect to
the sensor. The field of view of the sensor is 180, minimum range of the sensor is 0 and
maximum range of the sensor is 30, and that there are 721 elements of the ranges vector
that need to be divided into subsections. 
*/
// We initialize the range to be the threshold we got familiar with in Assignment 1.
float range = d_th;

for(i = 0; i < 721; i++){
    laser_scanning[i] = scanning -> ranges[i];
}



// Laser Scanning of the right side of the robot
for(i = 0; i < 143; i++){
    if(laser_scanning[i] < range) {
        range = laser_scanning[i];
    }
}
right = range;
range = d_th;

// Laser Scanning of the left side of the robot
for(i = 576; i < 713; i++){
    if(laser_scanning[i] < range) {
        range = laser_scanning[i];
    }
}
left = range;
range = d_th;

// Laser Scanning of the front side of the robot
for(i = 288; i < 431; i++){
    if(laser_scanning[i] < range) {
        range = laser_scanning[i];
    }
}
front = range;
range = d_th;
    
// For obstacle in front of the robot
if (front < 1.5){
    // In case there is an obstacle on the left
    if(left < right) {
       vel.linear.x = 0.2;
       vel.angular.z = -1.0;
    }
    
    // In case there is an obstacle on the right
    else if(right < left) {
       vel.linear.x = 0.2;
       vel.angular.z = 1.0;
    }
}
else // Drive straight
{
    vel.linear.x = manip_sp;
    vel.angular.z = 0.0;
    
    /* 
    Just like the first assignment, we need to take into consideration that there is
    a possibility that the robot will move backwards if something goes wrong with the
    computation. So we can create a simple if condition to counter that.
    */
    if(vel.linear.x < 0.0){
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
    }
}

// Towards the end of the function it needs to publish the /cmd_vel topic.
pub.publish(vel);


}

// Main Function
int main(int argc, char **argv){
/*
We need to first initialize the RobotControl node. There are 3 arguments that need to be passed on.
One is the argc, the second one is the argv and the final one is the name of the node itself. 
*/
ros::init(argc, argv, "robotcontrol_node");

/* NodeHandle is the main access point to communications with the ROS system. The first NodeHandle contructed will fully initialize this node, and the last NodeHandle destructed will close down the node.
*/
ros::NodeHandle n;

/*
The adverstise function is how you tell ROS that you want to publish on a given topic name. This invokes a call to the ROS master node, which keeps a registry of who is publishing and who is subscribing. After this advertise() call is made, the master node will notify anyone who is trying to subscribe to this topic name, and they will in turn negotiate a peer-to-peer connevtion with this node. advertise() returns a Publisher object which allows you to publish messages on that topic through a call to publish(). Once all copies of the returned Publisher object are destroyed, the topic will be automatically unadvertised.
The second parameter to advertise() is the size of the message queue used for publishing messages. If messages are published more quickly than we can send them, the number here specifies how many messages to buffer up before throwing some away.

In our case, we will set this parameter as 1 since we need to be size of the message queue to be once only. 
*/

// One of the tasks is for us to publish a velocity on the cmd_vel topic. Hence, here I have published it.
pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
ros::Subscriber sub=n.subscribe("/base_scan",1, laser_callback);
ch_in_sp = n.advertiseService("manipulate_speed",speed_response);
ros::spin(); 
return 0;
}
