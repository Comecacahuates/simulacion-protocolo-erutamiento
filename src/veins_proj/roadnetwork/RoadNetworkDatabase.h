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
 * @file RoadNetworkDatabase.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include <omnetpp.h>
#include <string>
#include <map>
#include "veins_proj/roadnetwork/RoadNetwork.h"
#include "veins_proj/geohash/GeohashLocation.h"

namespace veins_proj {

/*!
 * @brief Módulo global que contiene todas las redes viales
 * de la base de datos.
 */
class RoadNetworkDatabase: public omnetpp::cSimpleModule {

private:

    /*
     * Parámetros de configuración.
     */
    //! Directorio de la base de datos de archivos XML.
    std::string databaseDirectory;

    /*
     * Atributos.
     */
    //! Diccionario de redes viales.
    std::map<std::string, RoadNetwork*> roadNetworksMap;

public:

    /*
     * Destructor.
     */
    /*!
     * @brief Eliminar todas las redes viales almacenadas en el diccionario.
     */
    virtual ~RoadNetworkDatabase();

protected:

    /*
     * Interfaz del módulo.
     */
    /*!
     * @brief Inicialización.
     */
    virtual void initialize() override;
    /*!
     * @brief Manejo de mensajes.
     *
     * Este módulo no recibe ningún mensaje.
     *
     * @param message [in] Mensaje a procesar.
     */
    virtual void handleMessage(omnetpp::cMessage *message) override {
    }

public:

    /*
     * Acceso a los atributos.
     */
    /*!
     * @brief Acceso a una red vial.
     *
     * @param geohashLocation [in] Región Geohash cuya red vial se obtiene.
     * @return Red vial.
     */
    RoadNetwork* getRoadNetwork(const GeohashLocation &geohashLocation);
};

}    // namespace veins_proj
