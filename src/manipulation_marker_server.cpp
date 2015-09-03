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

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <manipulation_markers/UpdateMarker.h>
#include <std_msgs/Float64.h>
#include <manipulation_markers/WalkStart.h>

#include "manipulation_markers/manipulation_markers.h"
#include "manipulation_markers/manipulation_marker_server.h"

using namespace std;


ManipulationMarkerServer *manipulation_marker_server;
ManipulationMarkerServer *server;  // short name
ros::Publisher walk_pub;
tf::TransformListener *listener;


inline tf::StampedTransform get_transform(const string tf_name, const string parent="BODY") {
    tf::StampedTransform result;
    try {
        listener->waitForTransform(parent, tf_name, ros::Time(0), ros::Duration(1.0));
        listener->lookupTransform(parent, tf_name, ros::Time(0), result);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
    return result;
}


ManipulationMarkerServer::ManipulationMarkerServer() {
    int_marker_server.reset(new InteractiveMarkerServer("manipulation_marker", "", false));
}


ManipulationMarkerServer::~ManipulationMarkerServer() {
    int_marker_server.reset();
}


ManipulationMarker* ManipulationMarkerServer::add_marker(ManipulationMarker& init_marker) {
    const string marker_name = init_marker.name;
    markers[marker_name] = init_marker;  // copy the initial marker here
    ManipulationMarker& marker = markers[marker_name];  // important: add the internal copy
    int_marker_server->insert(marker.int_marker);
    int_marker_server->setCallback(marker.name, &process_marker_feedback);
    marker.menu.apply(*int_marker_server, marker.name);
    int_marker_server->applyChanges();
    return &markers[marker.name];
}


ContactMarker* ManipulationMarkerServer::add_contact_marker(ContactMarker& init_marker) {
    const string marker_name = init_marker.name;
    contact_markers[marker_name] = init_marker;  // copy the initial marker here
    ContactMarker& new_marker = contact_markers[marker_name];  // important: add the internal copy
    int_marker_server->insert(new_marker.int_marker);
    int_marker_server->setCallback(marker_name, &process_marker_feedback);
    new_marker.menu.apply(*int_marker_server, marker_name);
    int_marker_server->applyChanges();
    return &contact_markers[marker_name];
}


void ManipulationMarkerServer::remove_marker(const string& marker_name) {
    int_marker_server->erase(marker_name);
    int_marker_server->applyChanges();
    contact_markers.erase(marker_name);
    markers.erase(marker_name);
}


void ManipulationMarkerServer::update_marker(ContactMarker& marker) {
    //int_marker_server->erase(marker.name);
    //int_marker_server->applyChanges();

    // update marker pose from tf
    //ROS_INFO("updating marker, name is %s, tf origin x is %f", marker.name.c_str(), marker.tf.getOrigin().x());
    tf::pointTFToMsg(marker.tf.getOrigin(), marker.int_marker.pose.position);
    tf::quaternionTFToMsg(marker.tf.getRotation(), marker.int_marker.pose.orientation);

    int_marker_server->insert(marker.int_marker);
    //int_marker_server->setCallback(marker.name, &process_marker_feedback);
    marker.menu.apply(*int_marker_server, marker.name);
    //marker.menu.reApply(*int_marker_server);
    int_marker_server->applyChanges();
}


ContactMarker* ManipulationMarkerServer::find_marker(string marker_name) {
    if (markers.find(marker_name) != markers.end())
        return &markers[marker_name];
    if (contact_markers.find(marker_name) != contact_markers.end())
        return &contact_markers[marker_name];
    // store in dictionary if the loop below is too slow
    for (map<string, ManipulationMarker>::iterator manip_it = server->markers.begin(); manip_it != server->markers.end(); manip_it++)
        for (list<ContactMarker>::iterator contact_it = manip_it->second.contacts.begin(); contact_it != manip_it->second.contacts.end(); contact_it++)
            if (contact_it->name == marker_name)
                return &(*contact_it);
    return NULL;
}

geometry_msgs::Vector3Stamped wcmsg;
void process_marker_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    if (wheel_process_marker_feedback(feedback))
        return;

    ContactMarker* marker = server->find_marker(feedback->marker_name);
    if (!marker) {
        ROS_ERROR("[manipulation_markers] marker not found: %s", feedback->marker_name.c_str());
        return;
    }

    assert(feedback->marker_name == marker->name);
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
            marker->process_pose_feedback(feedback->pose);

    if (feedback->marker_name == "walk_dst_marker") {
        wcmsg.vector.x = feedback->pose.position.x;
        wcmsg.vector.y = feedback->pose.position.y;
        wcmsg.vector.z = atan2(2*feedback->pose.orientation.z*feedback->pose.orientation.w,
                feedback->pose.orientation.w*feedback->pose.orientation.w-feedback->pose.orientation.z*feedback->pose.orientation.z);//quatanion to euler
       // walk_pub.publish(wcmsg);
    }
    manipulation_marker_server->int_marker_server->applyChanges();
}




void switch_control_display(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
    ContactMarker* marker = server->find_marker(feedback->marker_name);
    MenuHandler::CheckState check_state;
    marker->menu.getCheckState(feedback->menu_entry_id, check_state);
    if (check_state == MenuHandler::CHECKED) {
        marker->set_dof_control(NO_CONTROL);
        marker->menu.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
    } else {
        marker->set_dof_control(SIX_DOF_CONTROL);
        marker->menu.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
    }
    marker->update();
}


void global_add_contact_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
    static int nb_new_contact_markers = 0;

    ManipulationMarker& parent_marker = server->markers[feedback->marker_name];

    string new_marker_name = "contact_marker_" + boost::lexical_cast<string>(++nb_new_contact_markers);

    tf::Vector3 frame_position(feedback->mouse_point.x, feedback->mouse_point.y, feedback->mouse_point.z);
    tf::Transform tf_parent_frame = get_transform(feedback->header.frame_id, parent_marker.name);
    tf::Vector3 parent_position = tf_parent_frame * frame_position;

    ContactMarker new_contact_marker(
            new_marker_name, parent_position, parent_marker.tf.getRotation(), 
            feedback->marker_name.c_str(), CONTACT_MARKER_SCALE);

    parent_marker.add_contact_marker(new_contact_marker);

    ostringstream s;
    ROS_INFO_STREAM(s.str() << ": new marker on " << feedback->marker_name << " at "
            << "\nposition = "
            << feedback->mouse_point.x
            << ", " << feedback->mouse_point.y
            << ", " << feedback->mouse_point.z
            << "\nframe: " << feedback->header.frame_id
            << " time: " << feedback->header.stamp.sec << "sec, "
            << feedback->header.stamp.nsec << " nsec");
}



void* thread_broadcast_markers(void *args) {
    tf::TransformBroadcaster br;
    ros::Rate rate(200);  // dt = 5 ms
    while (ros::ok()) {
        for (map<string, ContactMarker>::iterator contact_it = server->contact_markers.begin(); contact_it != server->contact_markers.end(); contact_it++) {
            ContactMarker& contact_marker = contact_it->second;
            contact_marker.broadcast_tf(br);
        }
        for (map<string, ManipulationMarker>::iterator it = server->markers.begin(); it != server->markers.end(); it++) {
            ManipulationMarker& manip_marker = it->second;
            manip_marker.broadcast_tf(br);
            for (list<ContactMarker>::iterator contact_it = manip_marker.contacts.begin(); contact_it != manip_marker.contacts.end(); contact_it++) {
                ContactMarker& contact_marker = *contact_it;
                contact_marker.broadcast_tf(br);
            }
        }
        if (wheel_reference_tf)
            br.sendTransform(tf::StampedTransform(*wheel_reference_tf, ros::Time::now(), "BODY", "wheel_center"));
        rate.sleep();
    }
}


void add_contact_marker(string marker_name, string init_tf) {
    tf::Transform tf = (init_tf.empty()) 
        ? tf::Transform(tf::Quaternion(0., 0., 0., 1.), tf::Vector3(.5, 0., -.25)) 
        : get_transform(init_tf);
    if (marker_name == "walk_dst_marker")  // kroon
        tf.setOrigin(tf::Vector3(0., 0., -.7)); 
    ContactMarker init_marker(marker_name, tf.getOrigin(), tf.getRotation(), "BODY");
    init_marker.set_color(0.1, 0.9, 0.1, 1.0);
    ContactMarker* new_marker = server->add_contact_marker(init_marker);
    new_marker->set_dof_control(SIX_DOF_CONTROL);
    new_marker->update();
}


bool main_handle_req(manipulation_markers::UpdateMarker::Request &req, manipulation_markers::UpdateMarker::Response &res) {
    if (req.command == "add_contact_marker") {
        add_contact_marker(req.marker, req.init_tf);
        return true;
    }

    if (req.command == "remove_marker") {
        server->remove_marker(req.marker);
        return true;
    }

    if (req.command == "set_marker_scale") {
        server->markers[req.marker].set_scale(req.scale);
        server->int_marker_server->applyChanges();
        return true;
    } 
    
    return false;
}


bool handle_req(manipulation_markers::UpdateMarker::Request &req, manipulation_markers::UpdateMarker::Response &res) {
    ROS_INFO("service call with command %s", req.command.c_str());

    if (main_handle_req(req, res))
        return true;
    if (polaris_handle_req(req, res))
        return true;
    if (wheel_handle_req(req, res))
        return true;
    if (masterarm_handle_req(req, res))
        return true;

    ROS_INFO("unknown command %s", req.command.c_str());
    return true;
}

bool walk_start(manipulation_markers::WalkStart::Request  &req,manipulation_markers::WalkStart::Response &res){
    ROS_INFO("service call with command %s", req.command.c_str());
	walk_pub.publish(wcmsg);
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "manipulation_markers");
    ros::NodeHandle n;

    ros::ServiceServer srv = n.advertiseService("manipulation_markers", handle_req);
    ros::ServiceServer walk_srv = n.advertiseService("walk_start", walk_start);
    ros::Duration(0.1).sleep();
    //walk_pub = n.advertise<geometry_msgs::Pose2D>("/HRP4/walking_cmd", 1000);

    // register object's ROS init here
    masterarm_init_ros(n);
    wheel_init_ros(n);

    walk_pub =  n.advertise<geometry_msgs::Vector3Stamped>("command/walking_cmd", 1000);

    listener = new tf::TransformListener();
    manipulation_marker_server = new ManipulationMarkerServer();
    server = manipulation_marker_server;

    pthread_t th;
    pthread_create(&th, NULL, thread_broadcast_markers, (void*)NULL);

    ros::spin();
}
