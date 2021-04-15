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
import veins_proj.networklayer.configurator.CarConfigurator;
import veins_proj.routing.CarRoutingProtocol;

//
// Módulo compuesto que representa un vehículo.
//
// @author Adrián Juárez
//
module Car extends AdhocHost
{
    parameters:
        @display("bgb=1253.47,916.05994");
        @display("i=misc/car");

        //# Parámetros
        hasIpv4 = false;
        hasIpv6 = true;
        **.isRouter = true;
        mobility.typename = "veins_proj.mobility.VehicleMobility";

    submodules:
        // Módulo del configurador de direcciones IPv6
        configurator: CarConfigurator {
            @display("p=123.28,719.44;is=s");
        }
        // Módulo del protocolo de enrutamiento
        routingProtocol: CarRoutingProtocol {
            @display("p=913,225");
        }

    connections:
        routingProtocol.ipOut --> tn.in++ if exists(routingProtocol);
        routingProtocol.ipIn <-- tn.out++ if exists(routingProtocol);
}
