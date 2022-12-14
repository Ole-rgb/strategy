#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/client/simple_action_client.h>
#include <hwp_goalplanner/subscriber_collection.h>
#include <hwp_goalplanner/state_machine.h>
#include <hwp_goalplanner/state.h>
#include <geometry_msgs/Twist.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

SubscriberCollection subs;
/**
 * * Gets the Map 
* Callbackfunction that gets the map from /map
* @param map The map  
*/
void getMap(nav_msgs::OccupancyGrid map){
    ROS_INFO("Recieving MAPDATA");
    cv::InputArray input =  cv::InputArray(map);
    //cv::OutputArray output = cv::OutputArray(cv::Mat(4,4,CV_8UC1));

    //cv::ximgproc::thinning(input,output);
    /**
     * TODO declare one global thinned map and always to the distancetransformation if the map updates (aka if getMAP is called)
    */
    //cv::imshow("original",input);
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
SubscriberCollection fillSubCollection(ros::NodeHandle nh){
    //Saves all subscribers in a SubscriberCollection
    subs.map_sub = nh.subscribe("/map", 10, getMap);
    subs.red_triangle_sub = nh.subscribe("/red_triangle_pos", 10, foundRedTriangle);
    subs.blue_squere_sub = nh.subscribe("/blue_square_pos", 10, foundBlueSquare);
    return subs;
}



int main(int argc, char **argv){
    ros::init(argc, argv, "GoalPlanner");
    ros::NodeHandle nh;
    SubscriberCollection subs = fillSubCollection(nh);
    ros::Publisher cmd_velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    StateMachine stateMachine(subs, cmd_velPub);
    stateMachine.run();

    ros::spin();
}