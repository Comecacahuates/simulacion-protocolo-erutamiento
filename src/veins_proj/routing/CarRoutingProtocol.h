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
 * @file CarRoutingProtocol.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include <omnetpp.h>
#include "inet/common/packet/Packet.h"
#include "inet/common/Ptr.h"
#include "inet/networklayer/common/InterfaceTable.h"
#include "inet/networklayer/contract/ipv6/Ipv6Address.h"
#include "inet/networklayer/contract/INetfilter.h"
#include "inet/networklayer/ipv6/Ipv6RoutingTable.h"
#include "veins_proj/networklayer/configurator/VehicleConfigurator.h"
#include "veins_proj/routing/RoutingProtocolBase.h"
#include "veins_proj/routing/Routing_m.h"
#include "veins_proj/mobility/VehicleMobility.h"
#include "veins_proj/roadnetwork/RoadNetworkDatabase.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "veins_proj/roadnetwork/ShortestPaths.h"
#include "veins_proj/util/ExpiringValuesMap.h"
#include <string>
#include <map>
#include <utility>

namespace veins_proj {

/*!
 * @brief Módulo que implementa las operaciones de enrutamiento
 * exclusivamnte para los vehículos.
 */
class CarRoutingProtocol: public RoutingProtocolBase {

protected:

    /*
     * Contexto.
     */
    //! Módulo de movilidad.
    VehicleMobility *mobility = nullptr;
    //! Módulo de configurador de interfaz.
    VehicleConfigurator *configurator = nullptr;

    /*
     * Mensajes propios.
     */
    //! Temporizador de transmisión de mensajes HOLA_VEHIC.
    omnetpp::cMessage *helloCarTimer;
    //! Temporizador de limpieza del directorio de *hosts* vecinos.
    omnetpp::cMessage *purgeNeighbouringHostsTimer;

    /*
     * Vehículos vecinos agrupados por arista.
     */
    //! Diccionario de vehículos vecinos arupados por arista.
    typedef std::multimap<Edge, inet::Ipv6Address> NeighbouringCarsByEdge;
    //! Iterador de diccionario de vehículos vecinos agrupados por arista.
    typedef NeighbouringCarsByEdge::iterator NeighbouringCarsByEdgeIt;
    //! Iterador de diccionario de vehículos vecinos agrupados por arista constante.
    typedef NeighbouringCarsByEdge::const_iterator NeighbouringCarsByEdgeConstIt;

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
     * @param [in] Etapa de inicialización.
     */
    virtual void initialize(int stage) override;

    /*
     * Manejo de mensajes.
     */
    /*!
     * @brief Manejo de mensajes propios.
     *
     * @param message [in] Mensaje a procesar.
     */
    virtual void processSelfMessage(omnetpp::cMessage *message) override;

    /*
     * Mensajes HOLA_VEHIC.
     */
    /*!
     * @brief Programar el temporizador de transmisión de mensajes HOLA_VEHIC.
     *
     * @param start [in] Indica si se va a programar el temporizador
     * a la hora de inicio.
     */
    virtual void scheduleHelloCarTimer(bool start = false);
    /*!
     * @brief Procesar el temporizador de transmisión de mensajes HOLA_VEIC.
     */
    virtual void processHelloCarTimer();
    /*!
     * @brief Crear mensaje HOLA_VEHIC.
     *
     * @param srcAddress [in] Dirección del vehículo que trnasmite el mensaje.
     * @return Mensaje HOLA_VEHIC.
     */
    virtual const inet::Ptr<HelloCar> createHelloCar(
            const inet::Ipv6Address &srcAddress) const;
    /*!
     * @brief Procesar mensaje HOLA_VEHIC.
     *
     * @param helloCar [in] Mensaje a procesar.
     */
    virtual void processHelloCar(const inet::Ptr<HelloCar> &helloCar) override;

    /*
     * Mensajes HOLA_HOST.
     */
    /*!
     * @brief Procesar mensaje HOLA_HOST.
     *
     * @param helloHost [in] Mensaje a procesar.
     */
    virtual void processHelloHost(const inet::Ptr<HelloHost> &helloHost)
            override;

    /*
     * Directorio de hosts vecinos.
     */
    /*!
     * @brief Diccionario de directorio de *hosts* vecinos.
     *
     * La clave es la dirección IPv6 del _host_ vecino, y el valor es el
     * registro de _host_ vecino.
     */
    typedef ExpiringValuesMap<inet::Ipv6Address, GeohashLocation> NeighbouringHosts;
    /*!
     * @brief Registro de diccionario de directorio de *hosts* vecinos.
     *
     * La clave es la dirección IPv6 del _host_ vecino, y el valor es el
     * registro de _host_ vecino.
     */
    typedef std::pair<inet::Ipv6Address, GeohashLocation> NeighbouringHost;
    /*!
     * @brief Iterador de registros para diccionario del directorio
     * de *hosts* vecinos.
     */
    typedef NeighbouringHosts::It NeighbouringHostsIt;
    /*!
     * @brief Iterador de registros para diccionario del directorio de *hosts*
     * vecinos constante.
     */
    typedef NeighbouringHosts::ConstIt NeighbouringHostsConstIt;
    /*!
     * @brief Directorio de *hosts* vecinos.
     */
    NeighbouringHosts neighbouringHosts;
    /*!
     * @brief Imrpimir el directorio de *hosts* vecinos.
     */
    virtual void showNeighbouringHosts() const;
    /*!
     * @brief Programar el temporizador de limpieza del directorio
     * de *hosts* vecinos.
     */
    virtual void schedulePurgeNeighbouringHostsTimer();
    /*!
     * @brief Procesar el temporizador de limpieza del directorio
     * de *hosts* vecinos.
     */
    virtual void processPurgeNeighbouringHostsTimer();

    /*
     * Enrutamiento.
     */
    /*!
     * @brief Verificar la cabecera de opciones de salto por salto.
     *
     * Verifica si la cabecera tiene la opción de ubicación del destino.
     *
     * Si el destino se encuentra en la misma subred, se verifica si
     * la cabecera contiene la opción de ubicación vial del destino. Si no
     * la tiene, la agrega.
     *
     * Si la cabecera no contiene la opción de vértices visitados, la agrega.
     */
    bool validateHopByHopOptionsHeader(inet::Packet *datagram) const;
    /*!
     * @brief Obtener el conjunto de aristas activas.
     *
     * Las aristas activas son aquellas en las que hay vehículos vecinos.
     *
     * @return Conjunto de aristas activas.
     */
    EdgeSet getActiveEdges() const;
    /*!
     * @brief Agrupar vehículos vecinos según la arista en la que se encuentran.
     *
     * @return Diccinario de vecinos agrupados según la aristan en la
     * que se encuentran.
     */
    NeighbouringCarsByEdge getNeighbouringCarByEdge() const;
    /*!
     * @brief Obtiene el tramo recto más largo desde el inicio de una ruta.
     *
     * @param shortestPath [in] Ruta de la que se obtiene el tramo recto.
     * @return Tramo recto más largo desde el inicio de la ruta.
     */
    VertexVector getStraightPath(const VertexVector &shortestPath,
            const ShortestPaths &shortestPaths) const;
    /*!
     * @brief Enrutar datagrama.
     *
     * Revisa si existe en la tabla de enrutamiento una ruta hacia la
     * dirección de destino. Si no existe, se intenta descubrir y crear una
     * ruta. Si no se encuentra la ruta, se descarta el datagrama.
     *
     * @param datagram [in] Datagrama a enrutar.
     * @return Resultado del enrutamiento.
     */
    virtual inet::INetfilter::IHook::Result routeDatagram(
            inet::Packet *datagram) override;
    /*!
     * @brief Enrutar datagrama hacia un *host* vecino.
     *
     * @param datagram [in] Datagrama a enrutar.
     * @return Resultado del enrutamiento.
     */
    inet::INetfilter::IHook::Result routeToNeighbouringHost(
            inet::Packet *datagram);
    /*!
     * @brief Enrutar datagrama hacia el vecino más cercano a una ubicación.
     *
     * @param datagram [in] Datagrama a enrutar.
     * @param geohashLocation [in] Ubicación hacia la que
     * se enruta el datagrama.
     * @return Resultado del enrutamiento.
     */
    inet::INetfilter::IHook::Result routeDatagramToLocation(
            inet::Packet *datagram, const GeohashLocation &geohashLocation);
    /*!
     * @brief Enrutar datagrama hacia una subred vecina.
     *
     * @param datagram  [in] Datagrama a enrutar.
     * @param adjacency [in] Adyacencia de la subred a la que se va
     * a enrutar el paquete.
     * @return Resultado del enrutamiento.
     */
    inet::INetfilter::IHook::Result routeDatagramToAdjacentNetwork(
            inet::Packet *datagram, GeohashLocation::Adjacency adjacency);
    /*!
     * @brief Enrutar el datagrama al vehículo vecino más lejano
     * en el tramo recto.
     *
     * @param datagram     [in] Datagrama a enrutar.
     * @param straightPath [in] Tramo recto hacia el que se va a enrutar.
     * @return Resultado del enrutamiento.
     */
    inet::INetfilter::IHook::Result routeDatagramFurthestInStraightPath(
            inet::Packet *datagram, const VertexVector &straightPath);
    /*!
     * @brief Enrutar el datagrama al vehículo vecino más cercano
     * en el tramo recto.
     *
     * @param datagram     [in] Datagrama a enrutar.
     * @param straightPath [in] Tramo recto hacia el que se va a enrutar.
     * @return Resultado del enrutamiento.
     */
    inet::INetfilter::IHook::Result routeDatagramClosestInStraightPath(
            inet::Packet *datagram, const VertexVector &straightPath);
    /*!
     * @brief Enrutar datagrama al vehículo vecino en la misma arista
     * que se encuentre más cerca a un vértice.
     *
     * @param datagram [in] Datagrama a enrutar.
     * @param vertex   [in] Vértice hacia el que se enruta el datagrama.
     * @return Resultado del enrutamiento.
     */
    inet::INetfilter::IHook::Result routeDatagramClosestToVertex(
            inet::Packet *datagram, const Vertex vertex);
    /*!
     * @brief Se obtiene el conjunto de vértices visitados.
     *
     * @param visitedVerticesOption [in] Opción de vértices visitados.
     * @return Conjunto de vértices visitados.
     */
    VertexSet getDatagramVisitedVertices(inet::Packet *datagram) const;
    /*!
     * @brief Obtener el vértice de destino local.
     *
     * @param datagram [in] Datagrama a enrutar.
     * @param shortestPath [in] Rutas más cortas.
     * @return Vértice de destino local y bandera que indica si sí se encontró.
     */
    std::pair<Vertex, bool> getLocalDestVertex(inet::Packet *datagram,
            const ShortestPaths &shortestPath) const;
    /*!
     * @brief Obtener vehículo vecino en la región Geohash adyacente.
     *
     * @param neighbouringGeohashRegion [in] Región Geohash adyacente.
     * @return Dirección IPv6 del siguiente salto, o `::/128`.
     * si no se encuentra ninguno.
     */
    const inet::Ipv6Address& findNextHopInAdjacentNetwork(
            const GeohashLocation::Adjacency adjacencyDirection) const;
    /*!
     * @brief Encontrar siguiente salto más cercano a una ubicación.
     *
     * @return Dirección IPv6 del siguiente salto, o `::/128`.
     * si no se encuentra ninguno.
     */
    const inet::Ipv6Address& findNextHopClosestToLocation(
            const GeohashLocation &geohashLocation) const;
    /*!
     * @brief Encontrar siguiente salto más lejano en el
     * tramo recto de la ruta.
     *
     * @param shortestPath  [in] Ruta vial a un vértice.
     * @param shortestPsths [in] Rutas viales más cortas
     * @return Dirección IPv6 del siguiente salto, o `::/128`.
     * si no se encuentra ninguno.
     */
    const inet::Ipv6Address& findNextHopFurthestInStraightPath(
            const VertexVector &straightPath) const;
    /*!
     * @brief Encontrar siguiente salto más cercano en el
     * tramo recto de la ruta.
     *
     * @param straightPath [in] Tramo recto de la ruta.
     * @return Dirección IPv6 del siguiente salto, o `::/128`.
     * si no se encuentra ninguno.
     */
    const inet::Ipv6Address& findNextHopClosestInStraightPath(
            const VertexVector &straightPath) const;
    /*!
     * @brief Buscar vehículo vecino más cercano a un vértice que
     * se encuentra en una arista.
     *
     * @param vertex [in] Vértice de referencia.
     * @param edge   [in] Arista en la que se encuentran los vehículos
     * entre los que se hace la búsqueda.
     * @return Dirección IPv6 del siguiente salto, o `::/128`.
     * si no se encuentra ninguno.
     */
    const inet::Ipv6Address& findNextHopClosestToVertex(const Vertex vertex,
            const Edge edge,
            const NeighbouringCarsByEdge &neighbouringCarsByEdge) const;
    /*!
     * @brief Buscar vehículo vecino más cercano a un vértice que
     * se encuentra en una arista.
     *
     * @param vertex [in] Vértice de referencia.
     * @param edge   [in] Arista en la que se encuentran los vehículos
     * entre los que se hace la búsqueda.
     * @return Dirección IPv6 del siguiente salto, o `::/128`.
     * si no se encuentra ninguno.
     */
    const inet::Ipv6Address& findNextHopClosestToVertex(
            const Vertex vertex) const;
    /*!
     * @brief Agrega los vértices visitados del siguiente salto.
     *
     * @param route        [in] Ruta a la que se agregan los vértices visitados
     * del siguiente salto.
     * @param shortestPath [in] Ruta vial.
     */
    VertexSet getNextHopVisitedVertices(const inet::Ipv6Address &nextHopAddress,
            const VertexVector &path) const;
    /*!
     * @brief Agrega los siguientes vértices visitados de la ruta a la opción
     * de vértices visitados del datagrama.
     *
     * @param datagram [in] Datagrama cuya opción de
     * vértices visitados se actualiza.
     * @param route    [in] Ruta que tomará el datagrama.
     */
    void updateTlvVisitedVerticesOption(inet::Packet *datagram,
            const inet::Ipv6Route *route) const;

    /*
     * Estatus del vehículo.
     */
    /*!
     * @brief Mostrar la dirección IPv6 del vehículo y su ubicación vial.
     */
    void showStatus() const;

    /*
     * Netfilter.
     */
    /*!
     * @brief Procesar datagrama recibido de la capa inferior
     * antes de enrutarlo.
     *
     * @param datagram [in] Datagrama a procesar.
     * @return Resultado del procesamiento.
     */
    virtual inet::INetfilter::IHook::Result datagramPreRoutingHook(
            inet::Packet *datagram) override;
    /*!
     * @brief Procesar datagrama recibido de la capa superior
     * antes de enrutarlo.
     *
     * @param datagram [in] Datagrama a procesar.
     * @return Resultado del procesamiento.
     */
    virtual inet::INetfilter::IHook::Result datagramLocalOutHook(
            inet::Packet *datagram) override;

    /*
     * Lifecycle.
     */
    virtual void handleStartOperation(inet::LifecycleOperation *operation)
            override;
    virtual void handleStopOperation(inet::LifecycleOperation *operation)
            override;
    virtual void handleCrashOperation(inet::LifecycleOperation *operation)
            override;

    /*
     * Notification.
     */
    virtual void receiveSignal(omnetpp::cComponent *source,
            omnetpp::simsignal_t signalID, omnetpp::cObject *obj,
            cObject *details) override;
};

}    // namespace veins_proj
