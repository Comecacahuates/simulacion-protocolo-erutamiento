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
 * @file RouteData.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include <omnetpp.h>
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"

namespace veins_proj {

class RouteData: public omnetpp::cObject {

protected:

    /*
     * Atributos.
     */
    //! Hora de expiración.
    omnetpp::simtime_t expiryTime;
    //! Siguientes vértices visitados en la ruta.
    VertexSet nextHopVisitedVertices;

public:

    /*
     * Constructor.
     */
    RouteData(const omnetpp::simtime_t expiryTime,
            const VertexSet nextHopVisitedVertices = {}) :
                    expiryTime(expiryTime),
                    nextHopVisitedVertices(nextHopVisitedVertices) {
    }

    /*
     * Acceso a los atributos.
     */
    /*!
     * @brief Acceso a la hora de expiración.
     *
     * @return Hora de expiración.
     */
    omnetpp::simtime_t getExpiryTime() const {
        return expiryTime;
    }
    /*!
     * @brief Acceso a los siguientes vértices visitados de la ruta
     *
     * @return
     */
    const VertexSet& getNextHopVisitedVertices() const {
        return nextHopVisitedVertices;
    }

    /*
     * Modificación de los atributos.
     */
    void setExpiryTime(const omnetpp::simtime_t expiryTime) {
        this->expiryTime = expiryTime;
    }
};

}    // namespace veins_proj
