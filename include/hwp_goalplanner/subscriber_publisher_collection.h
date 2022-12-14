#include <ros/ros.h>

#ifndef SUBSCRIBER_PUBLISHER_COLLECTION_H
#define SUBSCRIBER_PUBLISHER_COLLECTION_H

/**
 * * SubscriberPublisherCollection 
 * The struct contains  all subscribers  and publishers 
 * that are needed for the goalplanner.
*/
struct SubscriberPublisherCollection{
    ros::Subscriber map_sub;
    ros::Subscriber red_triangle_sub;
    ros::Subscriber blue_squere_sub;
    ros::Publisher cmd_velPub;
};

#endif