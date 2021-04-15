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

package veins_proj.networklayer.configurator;

import veins_proj.networklayer.configurator.ConfiguratorBase;

//
// Módulo de configurador de direcciones IPv6 para vehículos.
//
// @author Adrián Juárez
//
simple VehicleConfigurator extends ConfiguratorBase {
    
    parameters:
        @class(veins_proj::VehicleConfigurator);
        
        //# Parámetros
        // Tiempo de inicio de la configuración
		double startTime @unit(s) = default(5s);
        // Intervalo de actualización de la ubicación
		double locationUpdateInterval @unit(s) = default(1s);
}
