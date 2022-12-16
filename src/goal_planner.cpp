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
#include <hwp_goalplanner/state.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/* Create a collection that holds all subs and pubs */
SubscriberPublisherCollection SP_Collection;
cv::Mat output = cv::Mat();

/**
 * * Gets the Map 
* Callbackfunction that gets the map from /map
* @param map The map  
*/
void getMap(nav_msgs::OccupancyGrid msg){
    ROS_INFO("Recieving MAPDATA");
    ROS_INFO("Updating map");
    cv::Size sz(msg.info.height, msg.info.width);
    /** The node compiles and runs without any errors, 
     * ! but i cant show the image or access values ("Speicherzugriffsfehler (Speicherabzug geschrieben)")
    */
    cv::Mat input(sz, CV_8SC1, &(msg.data));
    cv::ximgproc::thinning(input, output, 0);

    





    //std::string mapMatWindow = "Map Matrix Image";
    //cv::namedWindow(mapMatWindow, cv::WINDOW_GUI_NORMAL);
    //cv::imshow(mapMatWindow, output);

    //cv::waitKey(0);
    //cv::destroyAllWindows();
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
 * * Fills the SubscriberPublisherCollection
 * @param nh The nodehandle thats needed for the subscribers
*/
void fillSPCollection(ros::NodeHandle nh){
    SP_Collection.map_sub = nh.subscribe("/map", 10, getMap);
    SP_Collection.red_triangle_sub = nh.subscribe("/red_triangle_pos", 10, foundRedTriangle);
    SP_Collection.blue_squere_sub = nh.subscribe("/blue_square_pos", 10, foundBlueSquare);
    SP_Collection.cmd_velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

    /**
     * * Used for the localisation spin of the roboter 
     * has to publish to /cmd_vel directly because move_base needs a location 
    */
    void initTurn(){
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
        twist.angular.z = 0;
        SP_Collection.cmd_velPub.publish(twist);
    }

    /**
     * Todo implement
    */
    void DistanceTransformation(){
        //cv::ximgproc::thinning(input, output);
    }


int main(int argc, char **argv){
    ros::init(argc, argv, "GoalPlanner");
    ros::NodeHandle nh;
    fillSPCollection(nh);
    /* The MBClient is responsable for sending movements via move_base */
    MBClient aC;
    aC.serverConnection();
    ROS_INFO("Finished initial setup");
    ROS_INFO("Localising");
    initTurn();

    ROS_INFO("Statemachine is running!");

    //aC.fullTurn();
    /**
     * TODO implement logic of the state machine here.
      * punkt aussuchen zum anfahren 
    */
    /*Distanztransformation*/
    
    ros::spin();
}