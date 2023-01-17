#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
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
#include <hwp_goalplanner/seach_strategy.h>

#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/* const used for the conversion of pixels to coordinates */
const double map_resolution = 0.025;
const double offset[] = {-2.5, -2.5, 0};

/*This array will hold all the fires found by the robot*/
// visualization_msgs::Marker red_triangle_markers[];
/*This array will hold all the humans found by the robot*/
// visualization_msgs::Marker blue_squere_markers[];

/* Create a collection that holds all subs and pubs */
SubscriberPublisherCollection SP_Collection;

/* The thinned map will be saved in this cd::Mat */
cv::Mat thinned = cv::Mat();

/* The seachStrategy handöse  */
SearchStrategy searchStrategy(0.2);

/**
 * * Gets the Map
 * Callbackfunction that gets the map from "/map" once at after the localisation
 * The method is responcable for the distancetransformation.
 * @param map The OccupanyGrid that contains all the important information about the map.
 */
void getMap(nav_msgs::OccupancyGrid msg)
{
    ROS_INFO("Recieving MAPDATA");
    cv::Size sz(msg.info.height, msg.info.width);
    cv::Mat input = cv::Mat::zeros(sz, CV_8UC1);

    for (int i = 0; i < msg.info.height; i++)
    {
        for (int j = 0; j < msg.info.width; j++)
        {
            if (msg.data[i * msg.info.width + j] == -1)
            {
                input.at<uchar>(j, i) = 0;
            }
            else if (msg.data[i * msg.info.width + j] == 0)
            {
                input.at<uchar>(j, i) = 255;
            }
            else if (msg.data[i * msg.info.width] > 0)
            {
                input.at<uchar>(j, i) = 255;
            }
        }
    }

    cv::ximgproc::thinning(input, thinned, cv::ximgproc::THINNING_ZHANGSUEN);
    /*
    std::cout << "input = " << std::endl << " "  << input << std::endl << std::endl;
    std::cout << "thinned = " << std::endl << " "  << thinned << std::endl << std::endl;
    cv::imshow("test_input", input);
    cv::imshow("test_thinned", thinned);
    cv::waitKey(0);
    */

    /* creates the vector that holds the points that should be visited by the robot */
    for (int i = 0; i < msg.info.height; i++)
    {
        for (int j = 0; j < msg.info.width; j++)
        {
            if (thinned.at<uchar>(i, j) == 255)
            {
                double x = i * map_resolution + offset[0];
                double y = j * map_resolution + offset[1];
                searchStrategy.getThinnedCoordinates()->push_back(Point(x, y));
            }
        }
    }
}

/**
 * * Gets a red triangle
 * Callbackfunction that gets a triangle
 * ! And fills the red_triangle_markers array.
 * @param fire The tiangle represents a fire
 */
void foundRedTriangle(visualization_msgs::Marker fire)
{
    ROS_WARN("Found red triangle");
}

/**
 * * Gets a blue squere
 * Callbackfunction that gets a squere
 * ! And fills the blue_squere_markers array.
 * @param human The squere represents a human
 */
void foundBlueSquare(visualization_msgs::Marker human)
{
    ROS_WARN("Found blue sqaure");
}

/**
 * * Fills the SubscriberPublisherCollection
 * @param nh The nodehandle thats needed for the subscribers and publishers
 */
void fillSPCollection(ros::NodeHandle nh)
{
    SP_Collection.map_sub = nh.subscribe("/map", 10, getMap);
    SP_Collection.red_triangle_sub = nh.subscribe("/red_triangle_pos", 10, foundRedTriangle);
    SP_Collection.blue_squere_sub = nh.subscribe("/blue_square_pos", 10, foundBlueSquare);
    SP_Collection.cmd_velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

/**
 * * Used locating the robot
 * has to publish to "/cmd_vel" directly because move_base needs a location
 */
void initTurn()
{
    /* "/cmd_vel" publishes a Twister msg */
    geometry_msgs::Twist twist;

    // no linear movement
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    // turns arounds the z axis
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
int main(int argc, char **argv)
{
    ros::init(argc, argv, "GoalPlanner");
    ros::NodeHandle nh;
    fillSPCollection(nh);
    MBClient aC;
    aC.serverConnection();
    ROS_INFO("Finished initial setup");
    ROS_INFO("Localising");
    initTurn();
    searchStrategy.UNITTESTING();
    ROS_INFO("Statemachine is running!");
    ros::spinOnce();

    /*For testing i created a publisher that publishes the posearray*/
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("/thinnedVector", 5);

    geometry_msgs::PoseArray points;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    geometry_msgs::Pose p1;

    while (!searchStrategy.getThinnedCoordinates()->empty())
    {
        /*Configure the TOPIC*/
        for (int i = 0; i < searchStrategy.getThinnedCoordinates()->size(); i++)
        {
            p1.position.x = searchStrategy.getThinnedCoordinates()->at(i).x;
            p1.position.y = searchStrategy.getThinnedCoordinates()->at(i).y;
            p1.position.z = 0;

            p1.orientation.x = 0;
            p1.orientation.y = 0;
            p1.orientation.z = 0;

            points.poses.push_back(p1);
        }
        pub.publish(points);
        ros::spinOnce();
        points.poses.clear();
        // ROS_INFO("Length VECTOR: %ld", searchStrategy.getThinnedCoordinates()->size());
        // Point closestPoint = searchStrategy.closestPoint(Point(aC.getPosition().transform.translation.x, aC.getPosition().transform.translation.y));
        // if (aC.driveTo(closestPoint))
        // {
        //     // successfully drove to the closestPoint
        //     searchStrategy.visited(closestPoint);
        // }
    }

    /*SEE IF IT WORKED*/
}
