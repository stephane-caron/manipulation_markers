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

#include "manipulation_markers/manipulation_marker_server.h"
#include "manipulation_markers/polaris.h"


void add_polaris() {
    ManipulationMarker *polaris_marker = new ManipulationMarker(
            "polaris_marker", DEFAULT_POLARIS_POS, DEFAULT_POLARIS_ROT, 
            DEFAULT_POLARIS_SCALE, "package://manipulation_markers/meshes/polaris.dae");
    polaris_marker->set_color(.5, .1, .1);

    // shift origin of interactive marker reference frame
    polaris_marker->int_marker.controls[0].markers[0].pose.position.x = -0.42; 
    polaris_marker->int_marker.controls[0].markers[0].pose.position.y = -0.00; 
    polaris_marker->int_marker.controls[0].markers[0].pose.position.z = -1.10; 

    // mesh dimensions are 1/1
    polaris_marker->int_marker.controls[0].markers[0].scale.x = 1.;
    polaris_marker->int_marker.controls[0].markers[0].scale.y = 1.;
    polaris_marker->int_marker.controls[0].markers[0].scale.z = 1.;

    manipulation_marker_server->add_marker(*polaris_marker);
}


bool polaris_handle_req(manipulation_markers::UpdateMarker::Request &req, manipulation_markers::UpdateMarker::Response &res) {
    if (req.command == "add_polaris") {
        add_polaris();
        return true;
    }

    return false;
}
