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
#include <strategy/subscriber_publisher_collection.h>
#include <strategy/mb_client.h>
#include <strategy/point.h>
#include <strategy/seach_strategy.h>
#include <sensor_msgs/PointCloud.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * TODO read directly from yaml ?
 */
/** const used for the conversion of pixels to coordinates 
 * ! Adjust to used MAP
*/
const double map_resolution = 0.025;
const double offset[] = {-2.0, -12.4, 0};
const double occupied_thresh = 0.65;
const double free_thresh = 0.196;

/*This vector will hold all the fires-marker found by the robot*/
std::vector<visualization_msgs::Marker> red_triangle_markers;
/*This vector will hold all the human-marker found by the robot*/
std::vector<visualization_msgs::Marker> blue_squere_markers;

/* Create a collection that holds all subs and pubs */
SubscriberPublisherCollection SP_Collection;

/* The thinned map will be saved in this cd::Mat */
cv::Mat thinned = cv::Mat();

/* The seachStrategy handles all interation with the thinnedCoordiantes  */
SearchStrategy searchStrategy(0.45);

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
            else if (msg.data[i * msg.info.width + j] <= free_thresh)
            {
                input.at<uchar>(j, i) = 255;
            }
            else if (msg.data[i * msg.info.width] >= occupied_thresh)
            {
                input.at<uchar>(j, i) = 0;
            }
        }
    }

    // std::cout << "input" << std::endl << " " << input << std::endl << std::endl;

    cv::ximgproc::thinning(input, thinned, cv::ximgproc::THINNING_ZHANGSUEN);

//    cv::imshow("INPUT", input);
//    cv::imshow("OUTPUT", thinned);
//    cv::waitKey(0);


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
 * TODO fill the red_triangle_markers array.
 * @param fire The tiangle represents a fire
 */
void foundRedTriangle(visualization_msgs::Marker fire)
{
    ROS_WARN("Found red triangle");
    red_triangle_markers.push_back(fire);
}

/**
 * * Gets a blue squere
 * Callbackfunction that gets a squere
 * TODO fill the blue_squere_markers array.
 * @param human The squere represents a human
 */
void foundBlueSquare(visualization_msgs::Marker human)
{
    ROS_WARN("Found blue sqaure");
    blue_squere_markers.push_back(human);
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
 * The method publishes directly to "/cmd_vel"
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

    /**
     * what a solution :d
    */
    for(int i = 0; i < 14; i++){
        SP_Collection.cmd_velPub.publish(twist);
        ros::Duration(0.5).sleep();
    }

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
    ROS_INFO("Localising");
    initTurn();
    ros::spinOnce();
    ROS_INFO("Statemachine is running!");

    /*For visualisation of all the points that have to be visited i created a publisher that publishes the pointcloud*/
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud>("/thinnedVector", 5);

    sensor_msgs::PointCloud points;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    geometry_msgs::Point32 p1;

    while (!searchStrategy.getThinnedCoordinates()->empty())
    {
        /*Configure the TOPIC*/
        for (int i = 0; i < searchStrategy.getThinnedCoordinates()->size(); i++)
        {
            p1.x = searchStrategy.getThinnedCoordinates()->at(i).x;
            p1.y = searchStrategy.getThinnedCoordinates()->at(i).y;
            p1.z = 0;

            points.points.push_back(p1);
        }
        pub.publish(points);
        ros::spinOnce();
        points.points.clear();

        Point closestPoint = searchStrategy.closestPoint(Point(aC.getPosition().transform.translation.x, aC.getPosition().transform.translation.y));

        if (aC.driveTo(closestPoint))
        {
            aC.fullTurn();
            searchStrategy.visited(closestPoint);

        }
    }

    /**
     * TODO handle Vision
     */
}
