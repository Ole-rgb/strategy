#include <ros/ros.h>
#include <hwp_goalplanner/state.h>
#include <hwp_goalplanner/subscriber_collection.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * * The state machine 
 * The class contains  all subscribers  
*/
class StateMachine{
    State current_state = CHECKING_SETUP;
    SubscriberCollection subCollection;
    MoveBaseClient actionClient;
    move_base_msgs::MoveBaseGoal goal;

    public:
        StateMachine(SubscriberCollection subCollection) : actionClient("/move_base", true){
            this->subCollection = subCollection;
        }

        void run(){
            ROS_INFO("Statemachine is running!");
            /*Initial setup*/
            tf2_ros::Buffer tfBuffer; 
            tf2_ros::TransformListener tfListener(tfBuffer);
            serverConnection();
            //for localisation
            fullTurn();


            /**
             * TODO implement logic of the state machine here.
             * position abfragen, 
             * distanztranstransform durchführen
             * punkt aussuchen zum anfahren 
            */

            geometry_msgs::TransformStamped transformStamped;
            try{
                transformStamped = tfBuffer.lookupTransform("map", "base_footprint", ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
            }
            ROS_INFO("Roboter Position: (%f, %f, 0)",
                    transformStamped.transform.translation.x,
                    transformStamped.transform.translation.y);

            //karte to cvmat
            //transformStamped.
        }   


    private: 
        /**
         * * Connects to the server (and waits for the connection)
        */
        void serverConnection(){
            //waiting for the server to connect
            while(!actionClient.waitForServer(ros::Duration(3.0))){
                ROS_INFO("Waiting for the server to come up");
            }
            ROS_INFO("Connected to server");
        }

        /** 
         * * Method for turning the roboter 360 degrees 
         * 
         */
        void fullTurn(){
            goal.target_pose.header.frame_id = "map";

            for(int i = 0; i < 3; ++i){
                goal.target_pose.header.stamp = ros::Time::now();

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
                    ROS_WARN("Goal not achieved");
                }
            }
        }
};

#endif