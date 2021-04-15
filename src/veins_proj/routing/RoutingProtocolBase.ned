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

//
// Módulo abstracto de base para los módulos del protocolo de enrutamiento.
//
// @author Adrián Juárez
//
simple RoutingProtocolBase like IManetRouting {

    parameters:
        @class(veins_proj::RoutingProtocolBase);
        @display("i=block/routing");

        //# Parámetros
        // Ruta del módulo de la base de datos de redes viales
		string roadNetworkDatabaseModule = default("roadNetworkDatabase");
		// Ruta del módulo de la tabla de enrutamiento
        string routingTableModule = default("^.ipv6.routingTable");
        // Ruta del módulo del protocolo IPv6
        string ipv6ProtocolModule = default("^.ipv6.ipv6");
        // Ruta del módulo de configuración de direcciones IPv6
        string configuratorModule = default("^.configurator");
        // Interfaz de salida
        string outputInterface = default("wlan0");
        
        //# Parámetros de enrutamiento
        // Tiempo de inicio del protocolo de enrutamiento
        double startTime @unit(s) = default(10s + uniform(0.0s, 3s));
        // Intervalo de mensajes HOLA-VEHIC
        volatile double helloCarInterval @unit(s) = default(3s + uniform(0.5s, 1.0s));
        // Tiempo de vigencia de los registros de vehículos vecinos
        double neighbouringCarValidityTime @unit(s) = default(5s);
        // Intervalo de mensajes HOLA-HOST
        volatile double helloHostInterval @unit(s) = default(3s + uniform(0.5s, 1.0s));
        // Tiempo de vigencia de los registros de <i>hosts</i> vecinos
        double neighbouringHostValidityTime @unit(s) = default(5s);
        // Tiempo de vigencia de las rutas
        double routeValidityTime @unit(s) = default(2s);
        // Tiempo de demora para transmisión de paquetes UDP consecutivos
        double udpPacketDelayTime @unit(s) = default(0.5s);

    gates:
        // Entrada para el protocolo IPv6
        input ipIn;
        // Salida para el protocolo IPv6
        output ipOut;
}
