/* Copyright (C) 2015 Stephane Caron <caron@phare.normalesup.org>
 *
 * This file is part of the manipulation_markers package.
 *
 * manipulation_markers is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * manipulation_markers is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * manipulation_markers. If not, see <http://www.gnu.org/licenses/>.
 */

#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>

#include "manipulation_markers/manipulation_marker_server.h"
#include "manipulation_markers/manipulation_markers.h"


ContactMarker *masterarm_left_marker = NULL;
ContactMarker *masterarm_left_ghost_marker = NULL;
ContactMarker *masterarm_right_marker = NULL;
ContactMarker *masterarm_right_ghost_marker = NULL;

ros::Subscriber masterarm_left_update;
ros::Subscriber masterarm_left_ghost_update;
ros::Subscriber masterarm_right_update;
ros::Subscriber masterarm_right_ghost_update;

double delta_scale = 1.0;


void masterarm_callback(const geometry_msgs::PoseStamped::ConstPtr& msg, ContactMarker *masterarm_marker, bool ignore_position=false, bool absolute_orientation=false) {
    if (!masterarm_marker)
        return;

    double dx = delta_scale * msg->pose.position.x;
    double dy = delta_scale * msg->pose.position.y;
    double dz = delta_scale * msg->pose.position.z;
    tf::Vector3 new_origin = masterarm_marker->tf.getOrigin() + tf::Vector3(dx, dy, dz);
    masterarm_marker->tf.setOrigin(new_origin);

    double x = msg->pose.orientation.x;
    double y = msg->pose.orientation.y;
    double z = msg->pose.orientation.z;
    double w = msg->pose.orientation.w;
    tf::Matrix3x3 Delta_A(tf::Quaternion(x, y, z, w));
    tf::Matrix3x3 A_prev = masterarm_marker->tf.getBasis();
    masterarm_marker->tf.setBasis(A_prev * Delta_A);

    masterarm_marker->update();
}


void masterarm_left_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    masterarm_callback(msg, masterarm_left_marker);
    masterarm_callback(msg, masterarm_left_ghost_marker);
}


void masterarm_right_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    masterarm_callback(msg, masterarm_right_marker);
    masterarm_callback(msg, masterarm_right_ghost_marker);
}


void masterarm_ghost_callback(const geometry_msgs::PoseStamped::ConstPtr& msg, ContactMarker *masterarm_ghost_marker) {
    double x = msg->pose.orientation.x;
    double y = msg->pose.orientation.y;
    double z = msg->pose.orientation.z;
    double w = msg->pose.orientation.w;
    masterarm_ghost_marker->tf.setRotation(tf::Quaternion(x, y, z, w));
    masterarm_ghost_marker->update();
}


void masterarm_left_ghost_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    masterarm_ghost_callback(msg, masterarm_left_ghost_marker);
}


void masterarm_right_ghost_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    masterarm_ghost_callback(msg, masterarm_right_ghost_marker);
}


void masterarm_init_ros(ros::NodeHandle& n) {
    masterarm_left_update = n.subscribe<geometry_msgs::PoseStamped>("/command/masterarm_left", 10, masterarm_left_callback);
    masterarm_right_update = n.subscribe<geometry_msgs::PoseStamped>("/command/masterarm_right", 10, masterarm_right_callback);
    masterarm_left_ghost_update = n.subscribe<geometry_msgs::PoseStamped>("/command/masterarm_left_ghost", 10, masterarm_left_ghost_callback);
    masterarm_right_ghost_update = n.subscribe<geometry_msgs::PoseStamped>("/command/masterarm_right_ghost", 10, masterarm_right_ghost_callback);
}


void add_masterarm() {
    tf::StampedTransform left_hand_center;
    tf::StampedTransform right_hand_center;
    tf::TransformListener listener;
    try {
        listener.waitForTransform("BODY", "left_hand_center", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("BODY", "left_hand_center", ros::Time(0), left_hand_center);
        listener.waitForTransform("BODY", "right_hand_center", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("BODY", "right_hand_center", ros::Time(0), right_hand_center);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }

    ContactMarker left_init("masterarm_left_marker", left_hand_center.getOrigin(), left_hand_center.getRotation(), "BODY", CONTACT_MARKER_SCALE);
    left_init.add_six_dof_control();
    left_init.set_color(0., .5, .5, 1.);
    masterarm_left_marker = manipulation_marker_server->add_contact_marker(left_init);

    ContactMarker left_ghost_init("masterarm_left_ghost_marker", left_hand_center.getOrigin(), left_hand_center.getRotation(), "BODY", CONTACT_MARKER_SCALE);
    left_ghost_init.add_six_dof_control();
    left_ghost_init.set_color(0., .5, .5, 1.);
    masterarm_left_ghost_marker = manipulation_marker_server->add_contact_marker(left_ghost_init);

    ContactMarker right_init("masterarm_right_marker", right_hand_center.getOrigin(), right_hand_center.getRotation(), "BODY", CONTACT_MARKER_SCALE);
    right_init.add_six_dof_control();
    right_init.set_color(0., .5, .5, 1.);
    masterarm_right_marker = manipulation_marker_server->add_contact_marker(right_init);

    ContactMarker right_ghost_init("masterarm_right_ghost_marker", right_hand_center.getOrigin(), right_hand_center.getRotation(), "BODY", CONTACT_MARKER_SCALE);
    right_ghost_init.add_six_dof_control();
    right_ghost_init.set_color(0., .5, .5, 1.);
    masterarm_right_ghost_marker = manipulation_marker_server->add_contact_marker(right_ghost_init);
}


bool masterarm_process_marker_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
        if (feedback->marker_name == "masterarm_left_marker") {
            masterarm_left_marker->process_pose_feedback(feedback->pose);
            masterarm_left_ghost_marker->process_pose_feedback(feedback->pose);
            return true;
        } else if (feedback->marker_name == "masterarm_right_marker") {
            masterarm_right_marker->process_pose_feedback(feedback->pose);
            masterarm_right_ghost_marker->process_pose_feedback(feedback->pose);
            return true;
        }
    }

    return false;
}


bool masterarm_handle_req(manipulation_markers::UpdateMarker::Request &req, manipulation_markers::UpdateMarker::Response &res) {
    if (req.command == "add_masterarm") {
        add_masterarm();
        return true;
    } 

    if (req.command == "remove_masterarm") {
        manipulation_marker_server->remove_marker("masterarm_marker");
        return true;
    } 

    if (req.command == "set_masterarm_scale") {
        delta_scale = req.scale;
        ROS_INFO("delta_scale = %f", delta_scale);
        return true;
    } 
    
    return false;
}
