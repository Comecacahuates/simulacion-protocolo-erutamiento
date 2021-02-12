//
// Copyright (C) 2017 Christoph Sommer <sommer@ccs-labs.org>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

package veins_proj.examples.test;

import inet.physicallayer.wireless.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.common.IntegratedMultiVisualizer;
import inet.visualizer.common.IntegratedVisualizer;
import org.car2x.veins.subprojects.veins_inet.VeinsInetManager;
import org.car2x.veins.subprojects.veins_proj.node.StaticHost;
import org.car2x.veins.subprojects.veins_proj.roadnetwork.RoadNetworkDatabase;
import org.car2x.veins.subprojects.veins_proj.locationservice.HostsLocationTable;
import org.car2x.veins.modules.obstacle.ObstacleControl;
import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import org.car2x.veins.base.modules.BaseWorldUtility;


network Scenario {
    parameters:
        @statistic[numRcvdPk](source=count(host[0].app[0].pingRxSeq); record=figure; targetFigure=numRcvdPkCounter; checkSignals=false);
        @figure[numRcvdPkCounter](type=indicatorText; pos=150,30; anchor=w; font=,12; textFormat="Received ping replies: %g"; initialValue=0);
        @statistic[numLost](source=count(host[0].app[0].numLost); record=figure; targetFigure=numLostPkCounter; checkSignals=false);
        @figure[numLostPkCounter](type=indicatorText; pos=150,70; anchor=w; font=,12; textFormat="Pings lost: %g"; initialValue=0);

    submodules:
        coordinateSystem: SimpleGeographicCoordinateSystem {
            @display("p=69,176");
        }
        visualizer: IntegratedMultiVisualizer {
            @display("p=69,475");
        }
        radioMedium: Ieee80211ScalarRadioMedium {
            @display("p=69,256");
        }
        manager: VeinsInetManager {
            @display("p=69,30");
        }
        hostsLocationTable: HostsLocationTable {
            @display("p=69,401");
        }
        obstacles: ObstacleControl {
            @display("p=69,330");
        }
        host[2]: StaticHost;
        roadNetworkDatabase: RoadNetworkDatabase {
            @display("p=69,103");
        }
        baseWorldUtility: BaseWorldUtility {
            playgroundSizeX = 1154m;
            playgroundSizeY = 609m;
            playgroundSizeZ = 10m;
            use2D = true;
            @display("p=69,553");
        }
}
