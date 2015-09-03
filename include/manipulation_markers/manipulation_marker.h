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

#ifndef __MANIPULATION_MARKER_H
#define __MANIPULATION_MARKER_H

#include "contact_marker.h"

using namespace interactive_markers;
using namespace std;
using namespace visualization_msgs;


struct ManipulationMarker : public ContactMarker {
    list<ContactMarker> contacts;

    inline ManipulationMarker() {};

    ManipulationMarker(string name, const tf::Vector3& position, 
            const tf::Quaternion& orientation, double scale=1., 
            const char* mesh_uri=NULL, DOF_CONTROL init_dof_control=NO_CONTROL);

    ContactMarker* add_contact_marker(ContactMarker&);
    void add_mesh_control(const char*);
    void set_scale(double);
    void update();
};


#endif // __MANIPULATION_MARKER_H
