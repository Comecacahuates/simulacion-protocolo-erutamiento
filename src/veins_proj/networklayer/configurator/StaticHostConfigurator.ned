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
// Módulo de configurador de direcciones IPv6 para <i>hosts</i>.
//
// @author Adrián Juárez
//
simple StaticHostConfigurator extends ConfiguratorBase {
    
    parameters:
        @class(veins_proj::StaticHostConfigurator);
        
        //# Parámetros
        // Ruta del módulo de la base de datos de redes viales
        string roadNetworkDatabaseModule = default("roadNetworkDatabase");
        // Ruta del módulo de la tabla de ubicaciones de los <i>hosts</i>
        string locationServiceModule = default("locationService");
}
