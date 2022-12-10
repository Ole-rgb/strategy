#include <ros/ros.h>

#ifndef SUBSCRIBER_COLLECTION_H
#define SUBSCRIBER_COLLECTION_H

/**
 * * SubscriberCollection 
 * The struct contains  all subscribers  
 * TODO public or private with getter/setter?
*/
struct SubscriberCollection{
    ros::Subscriber map_sub;
    ros::Subscriber red_triangle_sub;
    ros::Subscriber blue_squere_sub;
};

#endif