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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Utils{
    public: 
        ros::Subscriber map_sub;
        ros::Subscriber red_triangle_sub;
        ros::Subscriber blue_squere_sub;

        //Client ?

};

void getMap(nav_msgs::OccupancyGrid map){
    ROS_INFO("Gets Map");
}


void foundRedTriangle(visualization_msgs::Marker){
    ROS_INFO("Found red triangle");
}
void foundBlueSquare(visualization_msgs::Marker){
    ROS_INFO("Found blue sqaure");
}

int main(int argc, char **argv){
    ros::init(argc, argv, "GoalPlanner");
    ros::NodeHandle nh;
    Utils util;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Save all subscribers in the util instance
    util.map_sub = nh.subscribe("/map", 10, getMap);
    util.red_triangle_sub = nh.subscribe("/red_triangle_pos", 10, foundRedTriangle);
    util.blue_squere_sub = nh.subscribe("/blue_square_pos", 10, foundBlueSquare);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //setup Client (true causes the client to spin on its own)
    MoveBaseClient actionClient("/map", true);

    //waiting for the server to connect
    while(!actionClient.waitForServer(ros::Duration(3.0))){
        ROS_INFO("Waiting for the server to come up");
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    move_base_msgs::MoveBaseGoal goal;
   
    //configure the goal here

    //sending the goal
    ROS_INFO("Sending goal");
    actionClient.sendGoal(goal);


    actionClient.waitForResult();

    if(actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Goal achieved"); 
    }else{
        ROS_INFO("Goal not achieved");
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    ros::spin();
}