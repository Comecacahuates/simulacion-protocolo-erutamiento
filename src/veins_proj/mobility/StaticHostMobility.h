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
 * @file StaticHostMobility.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include <GeographicLib/GeoCoords.hpp>
#include <omnetpp.h>
#include "inet/mobility/static/StationaryMobility.h"
#include "inet/common/geometry/common/GeographicCoordinateSystem.h"
#include "veins_proj/geohash/GeohashLocation.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "veins_proj/roadnetwork/RoadNetworkDatabase.h"

namespace veins_proj {

/*!
 * @brief Módulo que representa la movilidad de un *host*.
 */
class StaticHostMobility: public inet::StationaryMobility {

private:

    /*
     * Contexto.
     */
    //! Módulo global de base de datos de redes viales.
    RoadNetworkDatabase *roadNetworkDatabase = nullptr;

    /*
     * Atributos.
     */
    //! Ubicación Geohash.
    GeohashLocation geohashLocation;
    //! Red vial en la que se encuentra el *host*.
    const RoadNetwork *roadNetwork = nullptr;

public:

    /*
     * Interfaz del módulo.
     */
    /*!
     * @brief Inicialización.
     *
     * @param stage [in] Etapa de inicialización.
     */
    virtual void initialize(int stage) override;

    /*
     * Acceso a los atributos.
     */
    /*!
     * @brief Acceso a la ubicación Geohash.
     *
     * @return Ubicación Geohash.
     */
    const GeohashLocation& getGeohashLocation() const {
        return geohashLocation;
    }
    /*!
     * @brief Acceso a la red vial.
     *
     * @return Red vial.
     */
    const RoadNetwork* getRoadNetwork() const {
        return roadNetwork;
    }
};

}    // namespace veins_proj
