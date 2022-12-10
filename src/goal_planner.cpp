#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/client/simple_action_client.h>
#include </home/oleroessler/catkin_ws/src/hwp_goalplanner/src/subscriber_collection.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * * Gets the Map 
 * Callbackfunction that gets the map from /map
 * @param map The map  
*/
void getMap(nav_msgs::OccupancyGrid map){
    ROS_INFO("Gets Map");
}

/**
 * * Gets a red triangle  
 * Callbackfunction that gets a triangle (is executed if a new(?) triangel is found).
 * @param fire The tiangle represents a fire  
*/
void foundRedTriangle(visualization_msgs::Marker fire){
    ROS_INFO("Found red triangle");
}

/**
 * * Gets a blue squere  
 * Callbackfunction that gets a squere (is executed if a new(?) squere is found).
 * @param human The squere represents a human  
*/
void foundBlueSquare(visualization_msgs::Marker human){
    ROS_INFO("Found blue sqaure");
}

/**
 * * Creates a filled SubscriberCollection instance
 * @param nh The nodehandle thats needed for the subscribers
 * @return A SubscriberCollection
*/
SubscriberCollection filledSubCollection(ros::NodeHandle nh){
    SubscriberCollection subs;
    //Saves all subscribers in a SubscriberCollection
    subs.map_sub = nh.subscribe("/map", 10, getMap);
    subs.red_triangle_sub = nh.subscribe("/red_triangle_pos", 10, foundRedTriangle);
    subs.blue_squere_sub = nh.subscribe("/blue_square_pos", 10, foundBlueSquare);

    return subs;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "GoalPlanner");
    ros::NodeHandle nh;
    SubscriberCollection subs = filledSubCollection(nh);

    //sets up Client (true causes the client to spin on its own)
    MoveBaseClient actionClient("/map", true);

    //waiting for the server to connect
    while(!actionClient.waitForServer(ros::Duration(3.0))){
        ROS_INFO("Waiting for the server to come up");
    }


    //creates goal to sent
    move_base_msgs::MoveBaseGoal goal;
   
    /* configure the goal here */

    //sending the goal
    ROS_INFO("Sending goal");
    actionClient.sendGoal(goal);

    //waiting for a responce
    while(!actionClient.waitForResult()){
        ROS_INFO("Waiting for responce");
    }

    //basic responce handling 
    if(actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Goal achieved"); 
    }else{
        ROS_INFO("Goal not achieved");
    }
    
    ros::spin();
}