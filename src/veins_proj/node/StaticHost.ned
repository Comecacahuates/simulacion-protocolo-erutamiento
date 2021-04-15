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

package veins_proj.node;

import inet.node.inet.AdhocHost;
import veins_proj.networklayer.configurator.StaticHostConfigurator;
import veins_proj.routing.StaticHostRoutingProtocol;

//
// Módulo compuesto que representa un <i>host</i>.
//
// @author Adrián Juárez
//
module StaticHost extends AdhocHost
{
    parameters:
        @display("i=device/cellphone;bgb=1253.7,823.725");

        hasIpv4 = false;
        hasIpv6 = true;
        **.isRouter = false;
        mobility.typename = "veins_proj.mobility.StaticHostMobility";

    submodules:
        // Módulo del configurador de direcciones IPv6
        configurator: StaticHostConfigurator {
            @display("p=124.424995,641.02496;is=s");
        }
        // Módulo del protocolo de enrutamiento
        routingProtocol: StaticHostRoutingProtocol {
            @display("p=913,225");
        }

    connections:
        routingProtocol.ipOut --> tn.in++ if exists(routingProtocol);
        routingProtocol.ipIn <-- tn.out++ if exists(routingProtocol);
}
