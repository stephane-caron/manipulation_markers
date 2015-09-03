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

#ifndef __POLARIS_H
#define __POLARIS_H

#include "manipulation_marker_server.h"

const double DEFAULT_POLARIS_SCALE = 0.4;
const tf::Vector3 DEFAULT_POLARIS_POS(0., -0.7, 0.);
const tf::Quaternion DEFAULT_POLARIS_ROT(0., 0., -0.7071, +0.7071);

bool polaris_handle_req(manipulation_markers::UpdateMarker::Request&, manipulation_markers::UpdateMarker::Response&);

#endif // __POLARIS_H
