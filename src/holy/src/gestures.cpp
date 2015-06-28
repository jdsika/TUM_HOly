/*
 * gestures.cpp
 *
 *  Created on: Oct 28, 2014
 *      Author: simon
 */

#include "kinect_scout/gestures.hpp"

double compute_distance(tf::StampedTransform &frame1, tf::StampedTransform &frame2) {
    return sqrt(pow(frame1.getOrigin().x()-frame2.getOrigin().x(),2)+pow(frame1.getOrigin().y()-frame2.getOrigin().y(),2)+pow(frame1.getOrigin().z()-frame2.getOrigin().z(),2));
}

double compute_norm(tf::StampedTransform &frame1) {
    return sqrt(pow(frame1.getOrigin().x(),2)+pow(frame1.getOrigin().y(),2)+pow(frame1.getOrigin().z(),2));
}

tf::StampedTransform compute_diff(tf::StampedTransform &frame1, tf::StampedTransform &frame2) {
    tf::StampedTransform result;
    // apply rotation matrix to frame 1
    frame1.setRotation(frame1.getRotation());
    result.getOrigin()=frame2.getOrigin()-frame1.getOrigin();
    return result;
}

enum Gesture recognize_gesture(tf::StampedTransform &left_hand,tf::StampedTransform &right_hand) {
    Gesture gesture;
    int gesture_count=0;
    // Recognize gestures:
    // Set Threshold, Half of maximum length (Later replaced by calibration)
    double threshold=0.55;
    // Left Hand LEFT
    if (left_hand.getOrigin().x()>threshold&&left_hand.getOrigin().y()<threshold&&left_hand.getOrigin().z()<threshold&&compute_norm(right_hand)<threshold) {
        gesture=LEFT;
        gesture_count++;
    }
    // left Hand UP
    if (left_hand.getOrigin().y()>threshold&&left_hand.getOrigin().x()<threshold&&left_hand.getOrigin().z()<threshold&&compute_norm(right_hand)<threshold) {
        gesture=UP_LEFT;
        gesture_count++;
    }
    // left Hand UP and right hand right
    if (left_hand.getOrigin().y()>threshold&&left_hand.getOrigin().x()<threshold&&left_hand.getOrigin().z()<threshold&&right_hand.getOrigin().x()<-threshold&&right_hand.getOrigin().y()>-threshold&&right_hand.getOrigin().z()>-threshold) {
        gesture=CORNER_LEFT;
        gesture_count++;
    }
    // left Hand Down
    if (left_hand.getOrigin().y()<(-threshold+0.3)&&left_hand.getOrigin().x()<threshold&&left_hand.getOrigin().z()<threshold&&compute_norm(right_hand)<(threshold-0.3)) {
        gesture=DOWN_LEFT;
        gesture_count++;
    }
    // left Hand in front
    if (left_hand.getOrigin().z()<-threshold&&left_hand.getOrigin().x()>-threshold&&left_hand.getOrigin().y()>-threshold&&compute_norm(right_hand)<threshold) {
        gesture=FORWARD_LEFT;
        gesture_count++;
    }
    // right Hand RIGHT
    if (right_hand.getOrigin().x()<-threshold&&right_hand.getOrigin().y()>-threshold&&right_hand.getOrigin().z()>-threshold&&compute_norm(left_hand)<threshold) {
        gesture=RIGHT;
        gesture_count++;
    }

    // right Hand UP
    if (right_hand.getOrigin().y()>threshold&&right_hand.getOrigin().x()<threshold&&right_hand.getOrigin().z()<threshold&&compute_norm(left_hand)<threshold) {
        gesture=UP_RIGHT;
        gesture_count++;
    }
    // right Hand DOWN
    if (right_hand.getOrigin().y()<(-threshold+0.3)&&right_hand.getOrigin().x()<threshold&&right_hand.getOrigin().z()<threshold&&compute_norm(left_hand)<(threshold-0.3)) {
        gesture=DOWN_RIGHT;
        gesture_count++;
    }
    // right Hand UP and left hand left
    if (right_hand.getOrigin().y()>threshold&&right_hand.getOrigin().x()<threshold&&right_hand.getOrigin().z()<threshold&&left_hand.getOrigin().x()>threshold&&left_hand.getOrigin().y()<threshold&&left_hand.getOrigin().z()<threshold) {
        gesture=CORNER_RIGHT;
        gesture_count++;
    }
    // right Hand in front
    if (right_hand.getOrigin().z()<-threshold&&right_hand.getOrigin().x()>-threshold&&right_hand.getOrigin().y()>-threshold&&compute_norm(left_hand)<threshold) {
        gesture=FORWARD_RIGHT;
        gesture_count++;
    }
    // Both Hands UP
    if (left_hand.getOrigin().y()>threshold&&left_hand.getOrigin().x()<threshold&&left_hand.getOrigin().z()<threshold&&right_hand.getOrigin().y()>threshold&&right_hand.getOrigin().x()<threshold&&right_hand.getOrigin().z()<threshold) {
        gesture=UP_BOTH;
        gesture_count++;
    }
    // Both Hands DOWN
    if (left_hand.getOrigin().y()<(-threshold+0.3)&&left_hand.getOrigin().x()<threshold&&left_hand.getOrigin().z()<threshold&&right_hand.getOrigin().y()<(-threshold+0.3)&&right_hand.getOrigin().x()<threshold&&right_hand.getOrigin().z()<threshold) {
        gesture=DOWN_BOTH;
        gesture_count++;
    }
    // Both Hands in front
    if (left_hand.getOrigin().z()<-threshold&&left_hand.getOrigin().x()>-threshold&&left_hand.getOrigin().y()>-threshold&&right_hand.getOrigin().z()<-threshold&&right_hand.getOrigin().x()>-threshold&&right_hand.getOrigin().y()>-threshold) {
        gesture=FORWARD_BOTH;
        gesture_count++;
    }
    // GOD, Left and right Hand
    if (right_hand.getOrigin().x()<-threshold&&right_hand.getOrigin().y()>-threshold&&right_hand.getOrigin().z()>-threshold&&left_hand.getOrigin().x()>threshold&&left_hand.getOrigin().y()<threshold&&left_hand.getOrigin().z()<threshold) {
        gesture=GOD;
        gesture_count++;
    }
    // if multiple gestures found return no gesture
    if (gesture_count==1)
        return gesture;
    else
        return NONE;
}

int gesture_is_left(tf::StampedTransform &right_hand) {
    double threshold=0.50;
    if (right_hand.getOrigin().x()<-threshold&&right_hand.getOrigin().y()>-threshold&&right_hand.getOrigin().z()>-threshold) {
        return 1;
    }
    else {
        return 0;
    }
}
