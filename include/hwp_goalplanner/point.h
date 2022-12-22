#ifndef POINT_H
#define POINT_H

/**
 * * A point the robot should drive to
*/
struct Point{
    double x;
    double y;
    bool visited = false;
    /*Default constructor*/
    Point(){}
    /** Constructor that sets x and y
     * @param x double that represents the x value of the point
     * @param y double that represents the y value of the point
    */
    Point(double x, double y){
        this->x = x;
        this->y = y;
    }
};

#endif