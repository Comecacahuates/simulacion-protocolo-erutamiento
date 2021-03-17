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

package veins_proj.sim;

import inet.physicallayer.wireless.common.contract.packetlevel.IRadioMedium;
import inet.visualizer.common.IntegratedMultiVisualizer;
import inet.visualizer.common.IntegratedVisualizer;
import inet.node.inet.INetworkNode;
import org.car2x.veins.subprojects.veins_inet.VeinsInetManager;
import veins_proj.roadnetwork.RoadNetworkDatabase;
import veins_proj.locationservice.HostsLocationTable;
import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import org.car2x.veins.base.modules.BaseWorldUtility;


network Scenario
{
    parameters:
        @statistic[numRcvdPk](source=count(host[0].app[0].pingRxSeq); record=figure; targetFigure=numRcvdPkCounter; checkSignals=false);
        @figure[numRcvdPkCounter](type=indicatorText; pos=150,30; anchor=w; font=,12; textFormat="Received ping replies: %g"; initialValue=0);
        @statistic[numLost](source=count(host[0].app[0].numLost); record=figure; targetFigure=numLostPkCounter; checkSignals=false);
        @figure[numLostPkCounter](type=indicatorText; pos=150,70; anchor=w; font=,12; textFormat="Pings lost: %g"; initialValue=0);
        
        int numHosts;

    submodules:
        coordinateSystem: SimpleGeographicCoordinateSystem {
            @display("p=69,176");
        }
        visualizer: IntegratedVisualizer {
            @display("p=69,475");
        }
        radioMedium: <default("inet.physicallayer.wireless.unitdisk.UnitDiskRadioMedium")> like IRadioMedium {
            @display("p=69,256");
        }
        manager: VeinsInetManager {
            @display("p=69,30");
        }
        hostsLocationTable: HostsLocationTable {
            @display("p=69,401");
        }
        host[numHosts]: <> like INetworkNode;
        roadNetworkDatabase: RoadNetworkDatabase {
            @display("p=69,103");
        }
        baseWorldUtility: BaseWorldUtility {
            @display("p=69,325");
        }
}
