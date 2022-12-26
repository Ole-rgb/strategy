#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/ximgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <hwp_goalplanner/subscriber_publisher_collection.h>
#include <hwp_goalplanner/mb_client.h>
#include <hwp_goalplanner/point.h>

#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/** 
 * * The points the robot needs to drive to to scan the entire map
 * ! This array has to be filled by the search-algorithm and not manually
*/
Point points[] = {Point(0, 0), Point(0, 1.2), Point(2, 1.2), Point(2,0), Point(1, 0.25)};

/*This array will hold all the fires found by the robot*/
//visualization_msgs::Marker red_triangle_markers[];
/*This array will hold all the humans found by the robot*/
//visualization_msgs::Marker blue_squere_markers[];

/* Create a collection that holds all subs and pubs */
SubscriberPublisherCollection SP_Collection;

/* The transformed map will be saved in this cd::Mat*/
cv::Mat output = cv::Mat();

/**
 * * Gets the Map 
* Callbackfunction that gets the map from "/map" once at after the localisation
* The method is responcable for the distancetransformation.
* @param map The OccupanyGrid that contains all the important information about the map.
*/
void getMap(nav_msgs::OccupancyGrid msg){
        
    ROS_INFO("Recieving MAPDATA");
    cv::Size sz(msg.info.height, msg.info.width);

    cv::Mat input = cv::Mat::zeros(sz, CV_8UC1);

    int row = 0;
    int column = 0;
    
    for(int i = 0; i < (msg.info.height*msg.info.width); i++){
        if(msg.data[i] == -1)
        {
            input.at<uchar>(row, column) = 0;  
        }
        else if(msg.data[i] == 0){
            //input.at<uchar>(row, column) = 255;  
            input.at<uchar>(row, column) = 0;  

        }
        else
        {
            if(msg.data[i] > 0){
                //input.at<uchar>(row, column) = 0;
                input.at<uchar>(row, column) = 255;  

            };
        };

        if(column == msg.info.width-1)
        {
            column = 0;
            row++;
        }else
        {
            column++; 
        };
   };


    cv::ximgproc::thinning(input, output, cv::ximgproc::THINNING_ZHANGSUEN);
    /*
    std::cout << "input = " << std::endl << " "  << input << std::endl << std::endl;
    std::cout << "output = " << std::endl << " "  << output << std::endl << std::endl;
    cv::imshow("test_input", input);
    cv::imshow("test_output", output);
    cv::waitKey(0);
    */
}

/**
 * * Gets a red triangle  
 * Callbackfunction that gets a triangle 
 * ! And fills the red_triangle_markers array.
 * @param fire The tiangle represents a fire  
*/
void foundRedTriangle(visualization_msgs::Marker fire){
    ROS_WARN("Found red triangle");
}

/**
 * * Gets a blue squere  
 * Callbackfunction that gets a squere  
 * ! And fills the blue_squere_markers array.
 * @param human The squere represents a human  
*/
void foundBlueSquare(visualization_msgs::Marker human){
    ROS_WARN("Found blue sqaure");
}

/**
 * * Fills the SubscriberPublisherCollection
 * @param nh The nodehandle thats needed for the subscribers and publishers
*/
void fillSPCollection(ros::NodeHandle nh){
    SP_Collection.map_sub = nh.subscribe("/map", 10, getMap);
    SP_Collection.red_triangle_sub = nh.subscribe("/red_triangle_pos", 10, foundRedTriangle);
    SP_Collection.blue_squere_sub = nh.subscribe("/blue_square_pos", 10, foundBlueSquare);
    SP_Collection.cmd_velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

/**
 * * Used locating the robot
 * has to publish to "/cmd_vel" directly because move_base needs a location 
*/
void initTurn(){
    /* "/cmd_vel" publishes a Twister msg*/
    geometry_msgs::Twist twist;
    
    //no linear movement 
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    //turns arounds the z axis
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 1;

    SP_Collection.cmd_velPub.publish(twist);

    /**
     * TODO find good z value and sleep value 
     * TODO test on robot
     */
    ros::Duration(7).sleep();

    /*Set the rotation around the z axis th 0 after the turn*/
    twist.angular.z = 0;
    SP_Collection.cmd_velPub.publish(twist);
}

/**
 ** The main function is responcable for running the entire GoalPlannerNode
 */
int main(int argc, char **argv){
    ros::init(argc, argv, "GoalPlanner");
    ros::NodeHandle nh;
    fillSPCollection(nh);
    MBClient aC;
    aC.serverConnection();
    ROS_INFO("Finished initial setup");
    ROS_INFO("Localising");
    initTurn();

    ROS_INFO("Statemachine is running!");
    ros::spinOnce();

    /**
     * TODO implement logic of the state machine here.
    */
    //As long as not all points are visited execute the loop
    while(!(points[0].visited && points[1].visited && points[2].visited && points[3].visited && points[4].visited)){
        Point *p_point;
        for(int i = 0; i <= 4; ++i){
            p_point = &points[i];
            if(!p_point->visited){
                ROS_INFO("Driving to Point: %d", i);
                break;
            }
        }
        aC.driveTo(p_point);
        aC.fullTurn();
        ros::spinOnce();
    }
    
}