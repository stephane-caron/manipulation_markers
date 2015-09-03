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

#ifndef __MANIPULATION_MARKER_SERVER_H
#define __MANIPULATION_MARKER_SERVER_H

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <manipulation_markers/UpdateMarker.h>

#include "manipulation_markers/contact_marker.h"
#include "manipulation_markers/manipulation_marker.h"

using namespace interactive_markers;
using namespace std;
using namespace visualization_msgs;


class ManipulationMarkerServer {
public:
    boost::shared_ptr<InteractiveMarkerServer> int_marker_server;
    map<string, ContactMarker> contact_markers;
    map<string, ManipulationMarker> markers;

    ManipulationMarkerServer();
    ~ManipulationMarkerServer();

    ManipulationMarker* add_marker(ManipulationMarker&);
    ContactMarker* add_contact_marker(ContactMarker&);
    void remove_marker(const string& marker_name);
    void update_marker(ContactMarker&);
    ContactMarker* find_marker(string marker_name);
};


void add_keyframe(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
void global_add_contact_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
void process_marker_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
void switch_control_display(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
void switch_yaw_control(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);

extern ManipulationMarkerServer *manipulation_marker_server;

#endif // __MANIPULATION_MARKER_SERVER_H
