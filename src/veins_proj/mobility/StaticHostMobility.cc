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

/*!
 * @file StaticHostMobility.cc
 * @author Adrián Juárez Monroy
 */

#include "veins_proj/mobility/StaticHostMobility.h"
#include "veins_proj/roadnetwork/RoadNetwork.h"
#include "inet/common/geometry/common/Coord.h"

using namespace veins_proj;

Define_Module(StaticHostMobility);

/*
 * Interfaz del módulo.
 */

/*!
 * @brief Inicialización.
 *
 * @param stage [in] Etapa de inicialización.
 */
void StaticHostMobility::initialize(int stage) {
    inet::StationaryMobilityBase::initialize(stage);

    /*
     * Etapa de inicialización local.
     */
    if (stage == inet::INITSTAGE_LOCAL) {
        /*
         * Contexto.
         */
        roadNetworkDatabase = omnetpp::check_and_cast<RoadNetworkDatabase*>(
                getModuleByPath(par("roadNetworkDatabaseModule")));
        if (!roadNetworkDatabase)
            throw omnetpp::cRuntimeError("No roadway database module found");
        /*
         * Etapa de inicialización de movilidad.
         */
    } else if (stage == inet::INITSTAGE_SINGLE_MOBILITY) {
        /*
         * Se obtiene la ubicación Geográfica a partir de la
         * ubicación en el lienzo y se actualiza.
         */
        inet::IGeographicCoordinateSystem *coordinateSystem =
                omnetpp::check_and_cast<inet::IGeographicCoordinateSystem*>(
                        getModuleByPath(par("coordinateSystemModule")));
        inet::Coord position = getCurrentPosition();
        inet::GeoCoord inetLocation =
                coordinateSystem->computeGeographicCoordinate(position);
        GeographicLib::GeoCoords location(inetLocation.latitude.get(),
                inetLocation.longitude.get());
        geohashLocation.setLocation(location);
        roadNetwork = roadNetworkDatabase->getRoadNetwork(geohashLocation);
    }
}
