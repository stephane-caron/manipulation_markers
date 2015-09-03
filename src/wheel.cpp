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

#include "manipulation_markers/manipulation_marker_server.h"
#include "manipulation_markers/manipulation_markers.h"


double cur_wheel_yaw = 0.;
ContactMarker *turning_marker = NULL;
ManipulationMarker *wheel_marker = NULL;
tf::Transform *wheel_reference_tf = NULL;
ros::Publisher wheel_yaw;
ros::Subscriber wheel_yaw_sub;


void wheel_init_ros(ros::NodeHandle& n) {
    wheel_yaw = n.advertise<std_msgs::Float32>("/wheel/yaw", 1000);
    wheel_yaw_sub = n.subscribe<std_msgs::Float32>("/wheel/yaw", 10, wheel_yaw_callback);
}


void add_wheel_turning_markers() {
    if (turning_marker)
        return;
    double scale_ratio = wheel_marker->int_marker.scale / 0.3;
    ContactMarker init_turning(
         "turning_marker", 
         scale_ratio * tf::Vector3(0.016, -0.116, -0.013), 
         tf::Quaternion(0., 0., 1., 0.), 
         "wheel_marker", CONTACT_MARKER_SCALE);
    turning_marker = wheel_marker->add_contact_marker(init_turning);
    wheel_marker->update();
    update_turning_marker();  // compute the marker's yaw
    ContactMarker interp1("interp1", 
            tf::Vector3(0.318, 0.410, 0.234),
            tf::Quaternion(0.173, -0.627, -0.319, 0.689), "BODY");
    manipulation_marker_server->add_contact_marker(interp1);
    ContactMarker interp2("interp2", 
            tf::Vector3(0.388, 0.285, 0.623),
            tf::Quaternion(0.595, -0.594, -0.367, 0.398), "BODY");
    manipulation_marker_server->add_contact_marker(interp2);
}


void update_wheel_local_transform() {
    tfScalar roll, pitch, yaw;
    tf::Transform Tw = wheel_reference_tf->inverseTimes(wheel_marker->tf);
    Tw.getBasis().getRPY(roll, pitch, yaw);
    cur_wheel_yaw = yaw;
}


void publish_cur_wheel_yaw() {
    std_msgs::Float32 msg;
    msg.data = cur_wheel_yaw;
    wheel_yaw.publish(msg);
}


void update_turning_marker() {
    /*
     * Update the contact marker's yaw rotation in the absolute world frame
     */

    // we don't use it for valve turning
    return;

    tfScalar roll, pitch, old_yaw, new_yaw;

    tf::Transform Tw = wheel_reference_tf->inverseTimes(wheel_marker->tf);
    tf::Transform Tl = Tw * turning_marker->tf;
    tf::Vector3 lhp = Tl.getOrigin();
    tf::Matrix3x3 Rl = Tl.getBasis();
    Rl.getRPY(roll, pitch, old_yaw);

    const double ymax0 = 0.13, wheel_scale0 = 0.4;
    double ymax = +ymax0 * wheel_marker->int_marker.scale / wheel_scale0;
    double ymin = -ymax0 * wheel_marker->int_marker.scale / wheel_scale0;
    double sign_x = (lhp.x() > 0) ? +1. : -1.;
    double y_frac = 2. * (ymax - (-lhp.y())) / (ymax - ymin);
    y_frac = max(0., min(1., y_frac));
    new_yaw = M_PI + sign_x * y_frac * 0.5;
    ROS_INFO("y_frac = %f\tnew_yaw = %f", y_frac, new_yaw);

    Rl.setRPY(roll, pitch, new_yaw);
    tf::Matrix3x3 R_marker = Tw.getBasis().inverse() * Rl;
    turning_marker->tf.setBasis(R_marker);
    turning_marker->update();
}


void wheel_yaw_callback(const std_msgs::Float32::ConstPtr& msg) {
    if (!wheel_marker || !wheel_reference_tf) {
        ROS_ERROR("[manipulation_markers] wheel reference is not set up");
        return;
    }

    tfScalar roll, pitch, yaw;

    //ROS_INFO("wheel yaw callback %f", msg->data);
    tf::Transform Tw = wheel_reference_tf->inverseTimes(wheel_marker->tf);
    tf::Matrix3x3 Rw = Tw.getBasis();
    Rw.getRPY(roll, pitch, yaw);

    if (fabs(msg->data - cur_wheel_yaw) < 1.e-2) {
        //ROS_INFO("ignored because %f %f %f", msg->data, cur_wheel_yaw, fabs(msg->data - cur_wheel_yaw));
        //ROS_INFO("%f", fmod(fabs(msg->data - cur_wheel_yaw), 2 * M_PI));
        return;
    }

    cur_wheel_yaw = msg->data;
    Rw.setRPY(roll, pitch, cur_wheel_yaw);
    Tw.setBasis(Rw);
    wheel_marker->tf = (*wheel_reference_tf) * Tw;
    wheel_marker->update();

    if (turning_marker)
        update_turning_marker();
}


void move_wheel_front() {
    wheel_marker->set_pose(DEFAULT_WHEEL_POS, DEFAULT_WHEEL_ROT);
}


void switch_yaw_control(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
    MenuHandler::CheckState check_state;
    wheel_marker->menu.getCheckState(feedback->menu_entry_id, check_state);
    if (check_state == MenuHandler::CHECKED) {
        wheel_marker->set_dof_control(NO_CONTROL);
        wheel_marker->menu.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
    } else {
        wheel_marker->set_dof_control(YAW_CONTROL);
        wheel_marker->menu.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
    }
    wheel_marker->update();
}


void set_wheel_reference() {
    wheel_reference_tf = new tf::Transform(wheel_marker->tf);
    add_wheel_turning_markers();
    cur_wheel_yaw = 0.;
    publish_cur_wheel_yaw();
    //wheel_marker->remove_dof_controls();
    //wheel_marker->update();
}


void set_wheel_reference_callback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) { 
    set_wheel_reference(); 
}


void add_wheel() {
    ManipulationMarker init_marker(
            "wheel_marker", DEFAULT_WHEEL_POS, DEFAULT_WHEEL_ROT, 
            DEFAULT_WHEEL_SCALE, "package://manipulation_markers/meshes/wheel.dae",
            NO_CONTROL);

    wheel_marker = manipulation_marker_server->add_marker(init_marker);

    //MenuHandler::EntryHandle wheel_menu = wheel_marker->menu.insert("Wheel");
    wheel_marker->menu.insert("Set wheel reference", &set_wheel_reference_callback);
    MenuHandler::EntryHandle entry = wheel_marker->menu.insert("Show yaw control", &switch_yaw_control);
    wheel_marker->menu.setCheckState(entry, MenuHandler::UNCHECKED);
    wheel_marker->update();
}


bool wheel_process_marker_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    if (feedback->marker_name != "wheel_marker")
        return false;

    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {

        wheel_marker->process_pose_feedback(feedback->pose);

        if (wheel_reference_tf) {
            update_wheel_local_transform();
            publish_cur_wheel_yaw();
            update_turning_marker();
        }
    }

    return true;
}


bool wheel_handle_req(manipulation_markers::UpdateMarker::Request &req, manipulation_markers::UpdateMarker::Response &res) {
    if (req.command == "add_wheel") {
        add_wheel();
        return true;
    } 
    
    if (req.command == "move_wheel_front") {
        move_wheel_front();
        return true;
    }

    if (req.command == "set_wheel_reference") {
        set_wheel_reference();
        return true;
    } 
    
    if (req.command == "set_wheel_scale") {
        wheel_marker->set_scale(req.scale);
        wheel_marker->update();
        return true;
    } 

    return false;
}


