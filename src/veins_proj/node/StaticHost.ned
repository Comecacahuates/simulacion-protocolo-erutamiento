//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package org.car2x.veins.subprojects.veins_proj.node;

import inet.node.inet.AdhocHost;
import org.car2x.veins.subprojects.veins_proj.networklayer.configurator.StaticHostConfigurator;
import org.car2x.veins.subprojects.veins_proj.networklayer.configurator.AddressCache;
import org.car2x.veins.subprojects.veins_proj.mobility.StaticHostMobility;
import org.car2x.veins.subprojects.veins_proj.routing.RoutingProtocolStaticHost;


module StaticHost extends AdhocHost
{
    parameters:
        @display("i=device/cellphone;bgb=1253.7,823.725");

        hasIpv4 = false;
        hasIpv6 = true;
        **.isRouter = false;
        mobility.typename = "org.car2x.veins.subprojects.veins_proj.mobility.StaticHostMobility";

    submodules:
        addressCache: AddressCache {
            @display("p=124.424995,727.64996");
        }
        hostConfigurator: StaticHostConfigurator {
            @display("p=124.424995,641.02496;is=s");
        }
        routingProtocol: RoutingProtocolStaticHost {
            @display("p=913,225");
        }

    connections:
        routingProtocol.ipOut --> tn.in++ if exists(routingProtocol);
        routingProtocol.ipIn <-- tn.out++ if exists(routingProtocol);
}
