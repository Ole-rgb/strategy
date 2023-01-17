#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <hwp_goalplanner/mb_client.h>
#include <hwp_goalplanner/point.h>
#include <cmath>

#ifndef SEARCH_STRATEGY_H
#define SEARCH_STRATEGY_H

/**
 * * The SearchStrategy class is responsible for all operation on the thinned-vector
 */
class SearchStrategy
{
private:
    /* data */
    std::vector<Point> thinnedCoordinates;
    double RADIUS;

    /**
     * * Calcuates the distance between the points
     * @param p1 The first point
     * @param p2 The second point
     * @return the distance between the points
     */
    double distance(Point p1, Point p2)
    {
        /**
         * TODO Needs to calc the distance between points based on the transformed map
         * ! not just dotproduct
         */
        /*Concern may not be valid because the distance isnt great !?*/
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

public:
    /* Constructors */
    SearchStrategy(double RADIUS);
    SearchStrategy();

    /**
     * * Finds the point that is closest to the robots position
     * @return The point that is the closest on the skeleton
     */
    Point closestPoint(Point robotPosition)
    {
        if (thinnedCoordinates.empty())
        {
            /**
             * TODO figure out what to do
            */
            return Point(-1, -1);
        };

        Point nearestPoint = thinnedCoordinates.at(0);
        for (int i = 1; i < thinnedCoordinates.size(); i++)
        {
            double currentDistance = distance(robotPosition, thinnedCoordinates.at(i));
            double prevSmallestDistance = distance(robotPosition, nearestPoint);

            if (currentDistance < prevSmallestDistance)
            {
                nearestPoint = thinnedCoordinates.at(i);
            };
        };
        return nearestPoint;
    }

    /**
     * * Sets all points in the radius to visited.
     * (aka pops all the points in the radius)
     */
    void visited(Point p)
    {
        for (int i = 0; i < thinnedCoordinates.size(); i++)
        {
            if (distance(thinnedCoordinates.at(i), p) < RADIUS)
            {
                thinnedCoordinates.erase(thinnedCoordinates.begin() + i);
            }
        };
    }

    std::vector<Point> *getThinnedCoordinates()
    {
        return &thinnedCoordinates;
    }

    void print(std::vector<Point> &input)
    {
        for (int i = 0; i < input.size(); i++) {
            printf("(");
            std::cout << input.at(i).x << ',';
            std::cout << input.at(i).y << ')';
            printf("\n");
        }
    }

    /*TESTING*/
    void UNITTESTING()
    {
        ROS_INFO("TESTCASE_1 - distance() -> Answer: 2; Method: %f", distance(Point(0, 0), Point(0, 2)));
        ROS_INFO("TESTCASE_2 - distance() -> Answer: 1,41; Method: %f", distance(Point(1, 0), Point(0, 1)));
        ROS_INFO("TESTCASE_3 - distance() -> Answer: 4,47 ; Method: %f", distance(Point(2, 1), Point(4, 5)));

 
        ROS_INFO("TESTCASE_1 - closestPoint(Point(1,0)) -> Answer: Point(1,0); Method: Point(%f,%f)", closestPoint(Point(1,0)).x, closestPoint(Point(1,0)).y);
        ROS_INFO("TESTCASE_2 - closestPoint(Point(2,0)) -> Answer: Point(2.025,0); Method: Point(%f,%f)", closestPoint(Point(2,0)).x, closestPoint(Point(2,0)).y);
        //ROS_WARN("TESTCASE_2 - closestPoint(Point(0,0)) -> Answer: Point(1,0); Method: Point(%f,%f)", closestPoint(Point(0,0)).x, closestPoint(Point(0,0)).y);



        ROS_WARN("TESTCASE_1 - visited(Point(1,0))");
        ROS_WARN("%ld vector-entries before visit",thinnedCoordinates.size());
        visited(Point(1,0));
        ROS_WARN("%ld vector-entries after visit",thinnedCoordinates.size());
    }
};

SearchStrategy::SearchStrategy(double RADIUS)
{
    this->RADIUS = RADIUS;
}

SearchStrategy::SearchStrategy()
{
    this->RADIUS = 0.2;
}
#endif