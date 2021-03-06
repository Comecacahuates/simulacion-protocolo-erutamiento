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

package veins_proj.routing;

import inet.routing.contract.IManetRouting;


simple RoutingProtocolBase like IManetRouting {

    parameters:
        @class(veins_proj::RoutingProtocolBase);
        @display("i=block/routing");

		// Parámetros de enrutamiento
		double startTime @unit(s) = default(10s + uniform(0.0s, 3s));
        volatile double helloCarInterval @unit(s) = default(3s + uniform(0.5s, 1.0s));
        double neighbouringCarValidityTime @unit(s) = default(5s);
        volatile double helloHostInterval @unit(s) = default(3s + uniform(0.5s, 1.0s));
        double neighbouringHostValidityTime @unit(s) = default(5s);
        double edgeStatusValidityTime @unit(s) = default(5s);
        double routeValidityTime @unit(s) = default(2s);
        double udpPacketDelayTime @unit(s) = default(0.5s);
        double vertexProximityRadius @unit(m) = default(10m);

		string roadNetworkDatabaseModule = default("roadNetworkDatabase");
        string routingTableModule = default("^.ipv6.routingTable");
        string networkProtocolModule = default("^.ipv6.ipv6");
        string configuratorModule = default("^.configurator");
        string outputInterface = default("wlan0");

    gates:
        input ipIn;
        output ipOut;
        input configuratorIn;
}
