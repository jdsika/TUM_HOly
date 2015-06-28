/*
 * gestures.hpp
 *
 *  Created on: Nov 18, 2014
 *      Author: simon
 */

#ifndef GESTURES_HPP_
#define GESTURES_HPP_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"

enum Gesture {NONE,LEFT,RIGHT,UP_LEFT,UP_RIGHT,FORWARD_LEFT,FORWARD_RIGHT, FORWARD_BOTH, UP_BOTH, GOD, DOWN_LEFT, DOWN_RIGHT, DOWN_BOTH, CORNER_RIGHT, CORNER_LEFT};

double compute_distance(tf::StampedTransform &frame1, tf::StampedTransform &frame2);
double compute_norm(tf::StampedTransform &frame1);
tf::StampedTransform compute_diff(tf::StampedTransform &frame1, tf::StampedTransform &frame2);
int gesture_is_left(tf::StampedTransform &right_hand);

enum Gesture recognize_gesture(tf::StampedTransform &left_hand,tf::StampedTransform &right_hand);

#endif /* GESTURES_HPP_ */
