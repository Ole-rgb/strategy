#include <ros/ros.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/ximgproc.hpp>
#include <hwp_goalplanner/point.h>

#ifndef MB_CLIENT_H
#define MB_CLIENT_H

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * * The MoveBaseClient class handles all iteraction with the move-base "module"
 *  It is responsable for creating a MoveBase Client and sending move_base_msgs.
*/
struct MBClient{
    MoveBaseClient actionClient;
    move_base_msgs::MoveBaseGoal goal;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    /** Default Constructor*/
    MBClient(): actionClient("/move_base", true), tfListener(tfBuffer){
        ROS_INFO("Actionclient created");
    }

    /**
     * * Connects to the server (and waits for the connection)
     * If the server doesnt come up the programm doesnt continue
    */
    void serverConnection(){
        //waiting for the server to connect
        while(!actionClient.waitForServer(ros::Duration(3.0))){
            ROS_INFO("Action client waits for the server to come up");
        }
        ROS_INFO("Action client connected to server");
    }

    /** 
     * * Method for turning the roboter 360 degrees 
     * The robots starts by turning to 0 degrees and then does 3 additional turns to forfill one 360 degrees turn
     * ! cant be used for initial localisation-spin
     * ! Rotation is relativ to the map and not to the robots rotation ()
     */
    void fullTurn(){
        goal.target_pose.header.frame_id = "map";

        for(int i = 0; i <= 3; ++i){
            geometry_msgs::TransformStamped position = getPosition();

            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = position.transform.translation.x;
            goal.target_pose.pose.position.y = position.transform.translation.y;
            goal.target_pose.pose.position.z = position.transform.translation.z;
                
            //use tf2 quaternions setRPY() method to rotate the robot
            tf2::Quaternion q;
            q.setRPY(0,0,i*2*M_PI/3);
            goal.target_pose.pose.orientation = tf2::toMsg(q);

            //sending the goal
            actionClient.sendGoal(goal);
            ROS_INFO("Goal is being exectuted: (%d/3 SPIN)", i);

            //waiting for a responce
            while(!actionClient.waitForResult(ros::Duration(1.0) )){
                ROS_INFO("Waiting for responce");
            }

            //basic responce handling 
            if(actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("Goal achieved");
            }else{
                ROS_WARN("Goal not achieved, something went wrong");
            }
        }
    }

    /**
     * * The function drives the robot to the handed location
     * If the point is successfully reached the points visited boolean
     * is set to true, else it remains false.
     * @param target a pointer to the target point.
    */
    void driveTo(Point *target){        
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = target->x;
        goal.target_pose.pose.position.y = target->y;
        goal.target_pose.pose.position.z = getPosition().transform.translation.z;
                
        //use tf2 quaternions setRPY() method to rotate the robot
        tf2::Quaternion q;
        q.setRPY(0,0,0);
        goal.target_pose.pose.orientation = tf2::toMsg(q);

        //sending the goal
        actionClient.sendGoal(goal);
        ROS_INFO("Goal is being exectuted");

        //waiting for a responce
        while(!actionClient.waitForResult(ros::Duration(1.0) )){
            ROS_INFO("Waiting for responce");
        }

        //basic responce handling 
        if(actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Goal achieved");
            target->visited = true;
        }else{
            ROS_WARN("Goal not achieved, something went wrong");
            target->visited = false;
        }
    }


    /**
     * *Gets the position of the robot on the map (in frame map)
     * ! not working (always going into the catch block)
     * @return geometry_msgs::TransformStamped (the position of the robot)
    */
    geometry_msgs::TransformStamped getPosition(){
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("map", "base_footprint", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            /**
             * ! All zeros lets the robot drive to (0,0,0)
             */
            ROS_WARN("%s",ex.what());
        }
        //prints the position out (for trouble shooting)
        ROS_INFO("Roboter Position: (%f, %f, 0)",
                transformStamped.transform.translation.x,
                transformStamped.transform.translation.y
        );
        return transformStamped;
    }
};

#endif