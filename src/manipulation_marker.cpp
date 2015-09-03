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

#include "manipulation_markers/manipulation_marker.h"
#include "manipulation_markers/manipulation_marker_server.h"

using namespace std;


ManipulationMarker::ManipulationMarker(
        std::string name, const tf::Vector3& position, const tf::Quaternion& orientation, 
        double scale, const char* mesh_uri, DOF_CONTROL init_dof_control)
    : ContactMarker(name, position, orientation, "BODY", scale, false) {
    assert(mesh_uri != NULL);
    add_mesh_control(mesh_uri);
    set_dof_control(init_dof_control);

    //MenuHandler::EntryHandle manip_menu = menu.insert("Manipulation");
    menu.insert("Add contact marker", &global_add_contact_marker);
}


void ManipulationMarker::add_mesh_control(const char* mesh_uri) { 
    const double mesh_to_int_marker_size = 2.5;

    Marker marker;
    marker.type = Marker::MESH_RESOURCE;
    marker.mesh_resource = mesh_uri;
    marker.scale.x = int_marker.scale * mesh_to_int_marker_size;
    marker.scale.y = int_marker.scale * mesh_to_int_marker_size;
    marker.scale.z = int_marker.scale * mesh_to_int_marker_size;
    marker.color.r = 0.9;
    marker.color.g = 0.6;
    marker.color.b = 0.3;
    marker.color.a = 0.7;

    InteractiveMarkerControl control;
    control.interaction_mode = InteractiveMarkerControl::MENU;
    control.name = "menu_control";
    control.always_visible = true;
    control.markers.push_back(marker);
    int_marker.controls.push_back(control);
}


void ManipulationMarker::set_scale(double scale) {
    double prev_scale = int_marker.scale;
    double scale_ratio = scale / prev_scale;

    int_marker.scale = scale;
    int_marker.controls[0].markers[0].scale.x = scale * 2.5;
    int_marker.controls[0].markers[0].scale.y = scale * 2.5;
    int_marker.controls[0].markers[0].scale.z = scale * 2.5;

    for (list<ContactMarker>::iterator it = contacts.begin(); it != contacts.end(); it++) {
        ContactMarker &contact = *it;
        contact.tf.setOrigin(scale_ratio * contact.tf.getOrigin());
        tf::pointTFToMsg(contact.tf.getOrigin(), contact.int_marker.pose.position);
        tf::quaternionTFToMsg(contact.tf.getRotation(), contact.int_marker.pose.orientation);
        contact.update();
    }

    this->update();
}


ContactMarker* ManipulationMarker::add_contact_marker(ContactMarker& contact_marker) {
    ROS_INFO("adding contact marker %s to %s", contact_marker.name.c_str(), name.c_str());
    contacts.push_back(contact_marker);
    manipulation_marker_server->int_marker_server->insert(contact_marker.int_marker);
    manipulation_marker_server->int_marker_server->setCallback(contact_marker.name, &process_marker_feedback);
    contact_marker.menu.apply(*manipulation_marker_server->int_marker_server, contact_marker.name);
    manipulation_marker_server->int_marker_server->applyChanges();
    return &contacts.back();
}


void ManipulationMarker::update() {
    ContactMarker::update();
    for (list<ContactMarker>::iterator contact_it = contacts.begin(); contact_it != contacts.end(); contact_it++)
        contact_it->update();
}
