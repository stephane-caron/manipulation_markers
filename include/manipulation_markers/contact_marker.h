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

#ifndef __CONTACT_MARKER_H
#define __CONTACT_MARKER_H

#include <geometry_msgs/Pose.h>
#include <interactive_markers/menu_handler.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

using namespace interactive_markers;
using namespace std;
using namespace visualization_msgs;

enum DOF_CONTROL { SIX_DOF_CONTROL, YAW_CONTROL, NO_CONTROL };

const double CONTACT_MARKER_SCALE = 0.15;


struct ContactMarker {
    string name;
    tf::Transform tf;
    InteractiveMarker int_marker;
    MenuHandler menu;

    inline ContactMarker() {}

    ContactMarker(string name, const tf::Vector3& position, 
            const tf::Quaternion& orientation, const string frame_id,
            double scale=CONTACT_MARKER_SCALE, bool add_controls=true);

    void add_box_control();
    void add_six_dof_control();
    void add_yaw_control();
    void broadcast_tf(tf::TransformBroadcaster&);
    void process_pose_feedback(geometry_msgs::Pose pose);
    void remove_dof_controls();
    void set_color(double r, double g, double b, double a=1.);
    void set_dof_control(DOF_CONTROL);
    void set_pose(const tf::Vector3&, const tf::Quaternion&);
    void update();
};


#endif // __CONTACT_MARKER_H
