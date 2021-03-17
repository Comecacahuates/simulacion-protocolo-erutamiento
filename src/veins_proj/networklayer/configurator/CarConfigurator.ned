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

simple CarConfigurator {
    
    parameters:
        @class(veins_proj::CarConfigurator);
        @display("i=block/cogwheel");
        
		double startTime @unit(s) = default(5s);
        string interface = default("wlan0");
		double locationUpdateInterval @unit(s) = default(1s);
}
