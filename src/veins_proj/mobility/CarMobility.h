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
 * @file CarMobility.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include <GeographicLib/GeoCoords.hpp>
#include <omnetpp.h>
#include "veins_inet/VeinsInetMobility.h"
#include "inet/common/lifecycle/LifecycleOperation.h"
#include "veins_proj/geohash/GeohashLocation.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "veins_proj/roadnetwork/RoadNetwork.h"
#include "veins_proj/roadnetwork/RoadNetworkDatabase.h"
#include <vector>
#include <algorithm>
#include <utility>

namespace veins_proj {

/*!
 * @brief Módulo que modela la movilidad de los vehículos.
 *
 * Contiene métodos para obtener la ubicación Geohash, velocidad,
 * dirección de movimiento, y más datos acerca del movimiento de un vehículo,
 * así como su relación con la red vial, como la región Geohash donde
 * se encuentra, su ubicación vial, si se encuentra en un vértice, etc.
 */
class CarMobility: public veins::VeinsInetMobility {

protected:

    /*
     * Parámetros de configuración.
     */
    //! Intervalo de actualización de la ubicación.
    omnetpp::simtime_t locationUpdateInterval;
    //! Distancia de proximidad a un vértice.
    double vertexProximityRadius;

    /*
     * Contexto
     */
    //! Módulo de base de datos de redes viales.
    RoadNetworkDatabase *roadNetworkDatabase = nullptr;

    /*
     * Datos de la ubicación.
     */
    //! Ubicación Geohash.
    GeohashLocation geohashLocation;
    //! Ubicación vial.
    LocationOnRoadNetwork locationOnRoadNetwork;
    //! Velocidad en metros por segundo.
    double speed;
    //! Ángulo acimutal de la dirección del movimiento en grados.
    double direction;
    //! Red vial en la que circula el vehículo.
    RoadNetwork *roadNetwork = nullptr;

    /*
     * Estatus.
     */
    //! Bandera de cambio de ubicación.
    bool locationChanged_;
    //! Bandera de cambio de arista.
    bool edgeChanged_;
    //! Bandera de cambio de región Geohash.
    bool regionChanged_;

    /*
     * Mensaje spropios.
     */
    //! Temporizador de actualización de la ubicación.
    omnetpp::cMessage *locationUpdateTimer;

public:

    /*!
     * @brief Destructor.
     */
    virtual ~CarMobility();

protected:

    /*
     * Interfaz del módulo.
     */
    /*!
     * @brief Número de etapas de inicialización.
     *
     * @return Número de etapas de inicialización.
     */
    virtual int numInitStages() const override {
        return inet::NUM_INIT_STAGES;
    }
    /*!
     * @brief Inicialización.
     *
     * @param stage [in] Etapa de inicialización.
     */
    virtual void initialize(int stage) override;
    /*!
     * @brief Manejo de mensajes.
     *
     * @param message [in] Mensaje a procesar.
     */
    virtual void handleMessage(omnetpp::cMessage *message) override;

public:

    /*
     * Acceso a los datos de ubicación.
     */
    /*!
     * @brief Acceso a la ubicación geográfica.
     *
     * @return Ubicación geográfica.
     */
    const GeographicLib::GeoCoords& getLocation() const {
        return geohashLocation.getLocation();
    }
    /*!
     * @brief Acceso a la ubicación Geohash.
     *
     * @return Ubicación Geohash.
     */
    const GeohashLocation& getGeohashLocation() const {
        return geohashLocation;
    }
    /*!
     * @brief Acceso a la ubicación vial.
     * @return
     */
    const LocationOnRoadNetwork& getLocationOnRoadNetwork() const {
        return locationOnRoadNetwork;
    }
    /*!
     * @brief Acceso a la velocidad.
     *
     * @return Velocidad en metros por segundo.
     */
    const double getSpeed() const {
        return speed;
    }
    /*!
     * @brief Acceso a la dirección de movimiento.
     *
     * @return Ángulo acimutal de la dirección del movimiento en grados.
     */
    const double getDirection() const {
        return direction;
    }
    /*!
     * @brief Acceso a la red vial.
     *
     * @return Red vial.
     */
    const RoadNetwork* getRoadNetwork() const {
        return roadNetwork;
    }

    /*
     * Acceso al estatus.
     */
    /*!
     * @brief Acceso a la bandera de cambio de ubicación.
     *
     * @return Bandera de cambio de ubicación.
     */
    bool locationChanged() const {
        return locationChanged_;
    }
    /*!
     * @brief Acceso a la bandera de cambio de arista.
     *
     * @return Bandera de cambio de arista.
     */
    bool edgeChanged() const {
        return edgeChanged_;
    }
    /*!
     * @brief Acceso a la bandera de cambio de región Geohash.
     *
     * @return Bandera de cambio de región Geohash.
     */
    bool regionChanged() const {
        return regionChanged_;
    }

    /*
     * Actualización de la ubicación.
     */
protected:
    /*!
     * @brief Programar el temporizador de actualización de la ubicación.
     */
    void scheduleLocationUpdateTimer();
    /*!
     * @brief Procesar el temporizador de actualización de la ubicacion.
     */
    void processLocationUpdateTimer();
public:
    /*!
     * @brief Actualizar la ubicación.
     */
    void updateLocation();
    /*!
     * @brief Verificar si el vehículo se encuentra en un vértice.
     *
     * @param vertex [in] Vértice de referencia.
     * @return `true` si el vehículo se encuentra en el vértice.
     *
     * TODO Eliminar.
     */
    bool isAtVertex(const Vertex vertex) const;
    /*!
     * @brief Verificar si el vehículo se encuentra en un vértice _gateway_.
     *
     * @return `true` si el vehículo se encuentra en un vértice _gateway_.
     * TODO Eliminar:
     */
    std::pair<Vertex, bool> isAtGateway() const;

private:

    /*!
     * @brief Actualizar la red vial si es necesario.
     *
     * @return `true` si se actualizó la red vial.
     */
    bool updateRoadNetwork();
};

}    // namespace veins_proj
