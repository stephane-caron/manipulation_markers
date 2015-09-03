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

#include <iostream>

#include "manipulation_markers/contact_marker.h"
#include "manipulation_markers/manipulation_marker_server.h"

using namespace std;


ContactMarker::ContactMarker(
        string name, const tf::Vector3& position, const tf::Quaternion& orientation, 
        const string frame_id, double scale, bool add_controls) 
    : name(name) {
    tf = tf::Transform(orientation, position);
    int_marker.header.frame_id = frame_id;

    tf::pointTFToMsg(position, int_marker.pose.position);
    tf::quaternionTFToMsg(orientation, int_marker.pose.orientation);
    int_marker.name = name;
    int_marker.scale = scale;

    if (add_controls)
        add_box_control();

    menu.insert(name);
    MenuHandler::EntryHandle move_entry = menu.insert("Show 6-DOF controls", &switch_control_display);
    menu.setCheckState(move_entry, MenuHandler::UNCHECKED);
}


void ContactMarker::add_box_control() {
    const double box_to_int_marker_size = 0.1;

    Marker marker;
    marker.type = Marker::CUBE;
    marker.scale.x = int_marker.scale * box_to_int_marker_size;
    marker.scale.y = int_marker.scale * box_to_int_marker_size;
    marker.scale.z = int_marker.scale * box_to_int_marker_size;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    InteractiveMarkerControl control;
    control.interaction_mode = InteractiveMarkerControl::MENU;
    control.name = "menu_control";
    control.always_visible = true;
    control.markers.push_back(marker);
    int_marker.controls.push_back(control);
}


void ContactMarker::add_six_dof_control() {
    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
}


void ContactMarker::add_yaw_control() {
    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
}


void ContactMarker::remove_dof_controls() {
    while (int_marker.controls.size() > 1)
        int_marker.controls.pop_back();
}


void ContactMarker::set_dof_control(DOF_CONTROL dof_control) {
    remove_dof_controls();
    switch (dof_control) {
        case SIX_DOF_CONTROL:
            add_six_dof_control();
            break;

        case YAW_CONTROL:
            add_yaw_control();
            break;

        default:
            break;
    }
}


void ContactMarker::set_color(double r, double g, double b, double a) {
    int_marker.controls[0].markers[0].color.r = r;
    int_marker.controls[0].markers[0].color.g = g;
    int_marker.controls[0].markers[0].color.b = b;
    int_marker.controls[0].markers[0].color.a = a;
}


void ContactMarker::set_pose(const tf::Vector3& position, const tf::Quaternion& orientation) {
    this->tf.setOrigin(position);
    this->tf.setRotation(orientation);
    this->update();
}


void ContactMarker::update() {
    manipulation_marker_server->update_marker(*this);
}


void ContactMarker::process_pose_feedback(geometry_msgs::Pose pose) {
    this->tf.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z)); 
    this->tf.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
    //tf::pointTFToMsg(this->tf.getOrigin(), int_marker.pose.position);
    //tf::quaternionTFToMsg(this->tf.getRotation(), int_marker.pose.orientation);
}


void ContactMarker::broadcast_tf(tf::TransformBroadcaster& br) {
    br.sendTransform(tf::StampedTransform(this->tf, ros::Time::now(), this->int_marker.header.frame_id, this->name));
}
