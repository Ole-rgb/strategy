#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <hwp_goalplanner/mb_client.h>
#include <hwp_goalplanner/point.h>

#ifndef SEARCH_STRATEGY_H
#define SEARCH_STRATEGY_H

/**
 * * The SearchStrategy class is responceable for all search algorithms
*/
class SearchStrategy
{
private:
    /* data */
    cv::Mat *p_transformed_map;
    Point searchQuere[];

    /* private methods*/
    /**
     * * Calcuates the distance between the points
     * @param p1 The first point
     * @param p2 The second point
     * @return the distance between the points
     */
    int distance(Point p, Point ){
        /**
         * TODO Needs to calc the distance between points based on the transformed map
         * !    not just dotproduct
         */
        return -1;
    }
public:
    /* Constructors */
    SearchStrategy(cv::Mat *p_transformed_map);
    ~SearchStrategy();

    /**
     * * Gets the closest point on the transformed map
     * only used once in the beginning of the programm 
     * @return The point that is the closest on the skeleton
    */
    Point initClosestPoint(){
        /**
         * TODO find closest point on the skeleton
         */
    }

    /**
     * Finds the next point that the robot should drive to
     */
    Point nextPoint(){
        /**
         * TODO   Wähle Punkt p(t+1) auf Skelett, sodass distance(p(t+1),p(t)) minimal ist und p(t+1) noch nicht visited ist
         */
    }
    
    /**
     * * Sets all points in the radius to visited.
     * @param radius the radius of the camera scan
     */
    void setVisited(int radius){
        /**
         * TODO set all points in radius r to visited
         */
    }


};

SearchStrategy::SearchStrategy(cv::Mat *p_transformed_map)
{
    this->p_transformed_map = p_transformed_map;
}

/*Default Constructor*/
SearchStrategy::~SearchStrategy() //: transformed_map() //we use a pointer to the map and not a map object!
{
    /*TODO*/
}

#endif