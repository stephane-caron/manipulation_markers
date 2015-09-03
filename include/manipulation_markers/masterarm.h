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

#ifndef __MASTERARM_H
#define __MASTERARM_H

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include "manipulation_marker_server.h"

bool masterarm_handle_req(manipulation_markers::UpdateMarker::Request&, manipulation_markers::UpdateMarker::Response&);
bool masterarm_process_marker_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
void masterarm_init_ros(ros::NodeHandle& n);
void masterarm_left_callback(const geometry_msgs::PoseStamped::ConstPtr&);
void masterarm_right_callback(const geometry_msgs::PoseStamped::ConstPtr&);

#endif // __MASTERARM_H
