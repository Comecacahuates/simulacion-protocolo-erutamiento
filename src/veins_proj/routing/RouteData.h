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
    //! Siguientes vértices visitados en la ruta.
    VertexVector nextVisitedVertices;

public:

    /*
     * Constructor.
     */
    RouteData(const VertexVector nextVisitedVertices) :
            nextVisitedVertices(nextVisitedVertices) {
    }

    /*
     * Acceso a los atributos.
     */
    /*!
     * @brief Acceso a los siguientes vértices visitados de la ruta
     * @return
     */
    const VertexVector& getNextVisitedVertices() const {
        return nextVisitedVertices;
    }
};

}    // namespace veins_proj
