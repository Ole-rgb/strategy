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
 * * The SearchStrategy class is responceable for all search algorithms
 */
class SearchStrategy
{
private:
    /* data */
    std::vector<Point> thinnedCoordinates;
    int RADIUS;

    /**
     * * Calcuates the distance between the points
     * @param p1 The first point
     * @param p2 The second point
     * @return the distance between the points
     */
    int distance(Point p1, Point p2)
    {
        /**
         * TODO Needs to calc the distance between points based on the transformed map
         * ! not just dotproduct
         */
        /*Concern may not be valid because the distance isnt great !?*/
        return sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2));
    }

public:
    /* Constructors */
    SearchStrategy(int RADIUS);
    SearchStrategy();

    /**
     * * Finds the point that is closest to the robots position
     * @return The point that is the closest on the skeleton
     */
    Point closestPoint()
    {
        /**
         * TODO find closest point on the skeleton
         */
        return Point(0,0);
    }

    /**
     * Finds the next point that the robot should drive to
     */
    Point nextPoint()
    {
        /**
         * TODO   Wähle Punkt p(t+1) auf Skelett, sodass distance(p(t+1),p(t)) minimal ist und p(t+1) noch nicht visited ist
         */
        return Point(0,0);

    }

    /**
     * * Sets all points in the radius to visited.
     */
    void visited()
    {
        /**
         * TODO pops all points in radius r
         */

    }

    std::vector<Point>* getThinnedCoordinates(){
        return &thinnedCoordinates;
    }
};

SearchStrategy::SearchStrategy(int RADIUS)
{
    this->RADIUS = RADIUS;
}

SearchStrategy::SearchStrategy()
{
    this->RADIUS = 1;
}
#endif