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

#ifndef __WHEEL_H
#define __WHEEL_H

#include <std_msgs/Float32.h>
#include <tf/tf.h>

#include "manipulation_marker_server.h"

const double DEFAULT_WHEEL_SCALE = .4;
const double MIN_WHEEL_SCALE = 0.2;
const double MAX_WHEEL_SCALE = 0.6;
const tf::Vector3 DEFAULT_WHEEL_POS(.5, 0., 0.3);
const tf::Quaternion DEFAULT_WHEEL_ROT(-0.52512, -0.478082, 0.521347, 0.473164);
const tf::Vector3 DEFAULT_TURNING_POS = DEFAULT_WHEEL_SCALE / 0.4 * tf::Vector3(0.212, -0.096, 0.088);
const tf::Quaternion DEFAULT_TURNING_ROT(-0.022, -0.074, -0.584, 0.808);

void update_turning_marker();
bool wheel_handle_req(manipulation_markers::UpdateMarker::Request&, manipulation_markers::UpdateMarker::Response&);
bool wheel_process_marker_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
void wheel_init_ros(ros::NodeHandle& n);
void wheel_yaw_callback(const std_msgs::Float32::ConstPtr&);

inline double wheel_scale_from_int(int k) {
    return MIN_WHEEL_SCALE + k * (MAX_WHEEL_SCALE - MIN_WHEEL_SCALE) / 100.;
}

inline int int_from_wheel_scale(double s) {
    return int((s - MIN_WHEEL_SCALE) * 100. / (MAX_WHEEL_SCALE - MIN_WHEEL_SCALE));
}

extern ros::Publisher wheel_yaw;
extern ros::Subscriber wheel_yaw_sub;
extern ros::Subscriber min_wrist_yaw_sub;
extern ros::Subscriber max_wrist_yaw_sub;
extern tf::Transform* wheel_reference_tf;

#endif // __WHEEL_H
