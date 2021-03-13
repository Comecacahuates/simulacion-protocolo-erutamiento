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
 * @file RoutingProtocolCar.h
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
#include "veins_proj/networklayer/configurator/CarConfigurator.h"
#include "veins_proj/routing/RoutingProtocolBase.h"
#include "veins_proj/routing/Routing_m.h"
#include "veins_proj/locationservice/HostsLocationTable.h"
#include "veins_proj/mobility/CarMobility.h"
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
class RoutingProtocolCar: public RoutingProtocolBase {

protected:

    /*
     * Contexto.
     */
    //! Módulo de movilidad.
    CarMobility *mobility = nullptr;
    //! Módulo de configurador de interfaz.
    CarConfigurator *configurator = nullptr;

    /*
     * Mensajes propios.
     */
    //! Temporizador de transmisión de mensajes HOLA_VEHIC.
    omnetpp::cMessage *helloCarTimer;
    //! Temporizador de limpieza del directorio de *hosts* vecinos.
    omnetpp::cMessage *purgeNeighbouringHostsTimer;
    //! Temporizador de limpieza de aristas activas.
    omnetpp::cMessage *purgeEdgesStatusTimer;
    //! Temporizador de limpieza de mensajes PONG pendientes.
    omnetpp::cMessage *purgePendingPongsTimer;

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
     */
    virtual void scheduleHelloCarTimer();
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
     * Mensajes PING.
     */
    /*!
     * @brief Crear mensaje PING.
     *
     * @param srcAddress [in] Dirección del vehículo remitente.
     * @param pingVertex [in] Vértice de origen.
     * @param pongVertex [in] Vértice de destino.
     * @return Mensaje PING.
     */
    virtual const inet::Ptr<Ping> createPing(
            const inet::Ipv6Address &srcAddress, Vertex pingVertex,
            Vertex pongVertex) const;
    /*!
     * @brief Procesar mensaje PING.
     *
     * Si el vehículo se encuentra en el vértice de destino,
     * se crea un mensaje PONG de respuesta y se transmite al vecino
     * que se encuentre más cerca del vértice de origen;
     * después, transmite un mensaje HOLA_VEHIC
     * indicando que la arista está activa.
     *
     * Si no se encuentra en el vértice de destino,
     * se busca el vehículo vecino más cercano vértice de destino
     * y se retransmite el mensaje hacia este.
     *
     * Si no se encuentra un vecino cerca del vértice de destino,
     * se crea un mensaje PONG de respuesta con la bandera de error activa,
     * y se busca el vecino que se encuentre más cerca del vertice
     * de origen para transmitir el mensaje hacia este.
     *
     * Si no se encuentra un vecino cerca del vértice de origen,
     * se descarta el mensaje.
     *
     * @param ping [in] Mensaje a procesar.
     */
    virtual void processPing(const inet::Ptr<Ping> &ping) override;

    /*
     * Mensajes PONG.
     */
    /*!
     * @brief Crear mensaje PONG.
     *
     * @param pingAddress [in] Dirección del vehículo que originó
     * el mensaje PING.
     * @param error [in] Bandera de error.
     * @param pingVertex [in] Vértice de origen.
     * @param pongVertex [in] Vértice de destino.
     * @return Mensaje PONG.
     */
    virtual const inet::Ptr<Pong> createPong(
            const inet::Ipv6Address &pingAddress, bool error, Vertex pingVertex,
            Vertex pongVertex) const;
    /*!
     * @brief Procesar mensaje PONG.
     *
     * Si la dirección de destino es una dirección local,
     * se revisa si es un mensaje PONG pendiente, en cuyo caso,
     * si el valor de la bandera E es falso,
     * se establece la arista correspondiente como arista activa,
     * y se programa el temporizador de limpieza de aristas activas.
     * También, se transmite un mensaje HOLA_VEHIC indicando que
     * la arista está activa.
     * Si se acabó el tiempo de espera del mensaje, se ignora el mensaje.
     *
     * Si la dirección de destino no es una dirección local, se verifica
     * si el destinatario se encuentra es un vecino. En ese caso,
     * se retransmite directamente hacia este. En otro caso, se busca un
     * vecino que se encuentre más cerca del vértice de destino.
     *
     * @param pong [in] Mensaje a procesar.
     */
    virtual void processPong(const inet::Ptr<Pong> &pong) override;

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
    typedef NeighbouringHosts::Iterator NeighbouringHostsIterator;
    /*!
     * @brief Iterador de registros para diccionario del directorio de *hosts*
     * vecinos constante.
     */
    typedef NeighbouringHosts::ConstIterator NeighbouringHostsConstIterator;
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
     * Estatus de las aristas.
     */
    /*!
     * @brief Diccionario de aristas activas.
     *
     * La clave es la arista activa, y el valor es la hora de expiración
     * del registro.
     */
    typedef ExpiringValuesMap<Edge, bool> EdgesStatus;
    /*!
     * @brief Valor.
     */
    typedef EdgesStatus::MapValue EdgeStatus;
    /*!
     * @brief Iterador para diccionario de aristas activas.
     */
    typedef EdgesStatus::Iterator EdgesStatusIterator;
    /*!
     * @brief Iterador para diccionario de aristas activas constante.
     */
    typedef EdgesStatus::ConstIterator EdgesStatusConstIterator;
    /*!
     * @brief Estatus de las aristas.
     *
     * Cuando se recibe un mensaje PONG indicando que la arista se
     * encuentra activa, se agrega un registro.
     */
    EdgesStatus edgesStatus;
    /*!
     * @brief Imprimir las aristas activas.
     */
    virtual void showEdgesStatus() const;
    /*!
     * @brief Programar el temporizador de limpieza de aristas activas.
     */
    virtual void schedulePurgeEdgesStatusTimer();
    /*!
     * @brief Procesar el temporizador de limpieza de aristas activas.
     */
    virtual void processPurgeEdgesStatusTimer();

    /*
     * Datagramas demorados.
     */
    /*!
     * @brief Diccionario de datagramas demorado.
     *
     * La clave es la dirección de destino del datagrama, y el valor es
     * el datagrama.
     */
    typedef std::multimap<Edge, inet::Packet*> DelayedDatagrams;
    /*!
     * @brief Iterador para diccionario de datagramas demorados.
     */
    typedef DelayedDatagrams::iterator DelayedDatagramsIterator;
    /*!
     * @brief Iterador para diccionario de datagramas demorados constante.
     */
    typedef DelayedDatagrams::const_iterator DelayedDatagramsConstIterator;
    /*!
     * @brief Paquetes demorados.
     */
    DelayedDatagrams delayedDatagrams;
    /*!
     * @brief Imprimir los datagramas demorados.
     */
    void showDelayedDatagrams() const;
    /*!
     * @brief Eliminar los datagramas demorados cuyo mensaje PONG no llegó,
     * o no existe una ruta para enviarlos.
     */
    void removeStuckDelayedDatagrams();

    /*
     * Operación ping-pong
     *
     */
    /*!
     * @brief Iniciar una operación ping-pong.
     *
     * Se crea un mensaje PING y se transmite hacia el vehículo vecino
     * más cercano al vértice de destino.
     *
     * @param pingVertex [in] Vértice de origen.
     * @param pongVertex [in] Vértice de destino.
     * @return `true` si se pudo iniciar la operación ping-pong.
     */
    bool startPingPong(const Vertex pingVertex, const Vertex pongVertex);

    /*
     * Mensajes PONG pendientes.
     */
    /*!
     * @brief Registro de mensaje PONG pendiente.
     */
    struct PendingPongValue {
        /*!
         * @brief Vértice de origen.
         */
        Vertex pingVertex;
        /*!
         * @brief Vértice de destino.
         */
        Vertex pongVertex;
    };
    /*!
     * @brief Diccionario de mensajes PONG pendientes.
     *
     * La clave es la arista por la que se envió el mensaje PING,
     * y el valor es el par de vértices de origen y destino.
     */
    typedef ExpiringValuesMap<Edge, PendingPongValue> PendingPongs;
    /*!
     * @brief Valor.
     */
    typedef PendingPongs::MapValue PendingPong;
    /*!
     * @brief Iterador para diccionario de mensajes PONG pendientes.
     */
    typedef PendingPongs::Iterator PendingPongsIterator;
    /*!
     * @brief Iterador para diccionario de mensajes PONG pendientes constante.
     */
    typedef PendingPongs::ConstIterator PendingPongsConstIterator;
    /*!
     * @brief Mensajes PONG pendientes.
     */
    PendingPongs pendingPongs;
    /*!
     * @brief Imprimir los mensajes PONG pendientes.
     */
    void showPendingPongs() const;
    /*!
     * @brief Programar el temporizador de limpieza de mensajes
     * PONG pendientes.
     */
    void schedulePurgePendingPongsTimer();
    /*!
     * @brief Procesar el temporizador de limpieza de mensajes
     * PONG pendientes.
     */
    void processPurgePendingPongsTimer();

    /*
     * Enrutamiento.
     */
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
    virtual bool validateHopByHopOptionsHeader(inet::Packet *datagram) const;
    /*!
     * @brief Se obtiene el conjunto de vértices visitados.
     *
     * @param visitedVerticesOption [in] Opción de vértices visitados.
     * @return Conjunto de vértices visitados.
     */
    virtual VertexSet getVisitedVertices(inet::Packet *datagram) const;
    /*!
     * @brief Obtener el vértice de destino local.
     *
     * @param datagram [in] Datagrama a enrutar.
     * @param shortestPath [in] Rutas más cortas.
     * @return Vértice de destino local y bandera que indica si sí se encontró.
     */
    virtual std::pair<Vertex, bool> getLocalDestVertex(inet::Packet *datagram,
            const ShortestPaths &shortestPath) const;
    /*!
     * @brief Obtener aristas en la ruta más corta que forman un tramo recto.
     *
     * Se obtienen las aristas en la ruta que forman el tramo largo más recto,
     * y en las que haya vehículos vecinos circulando.
     *
     * @param shortestPathToDestVertex [in] Ruta más corta al vértice
     * de destino.
     * @param shortestPath [in] Rutas más cortas.
     * @return Aristas que forman un
     *
     * TODO Eliminar.
     */
    virtual EdgeVector getReachableEdges(
            const VertexVector &shortestPathToDestVertex,
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
     * @brief Encontrar siguiente salto más lejano en el tramo más recto
     * de la ruta vial.
     *
     * @param shortestPath  [in] Ruta vial a un vértice.
     * @param shortestPsths [in] Rutas viales más cortas
     * @return Dirección IPv6 del siguiente salto, o `::/128`.
     * si no se encuentra ninguno.
     */
    const inet::Ipv6Address& findNextHopFurthestInStraightPath(
            const VertexVector &shortestPath,
            const ShortestPaths &shortestPaths) const;
    /*!
     * @brief Buscar vehículo vecino más cercano a un vértice que
     * se encuentra en la misma arista.
     *
     * Se buscan los vehículos vecinos que circulan sobre la misma arista,
     * y se obtiene el que se encuentra a la menor distancia del vértice
     * indicado.
     *
     * @param vertex [in] Vértice de referencia.
     * @return Dirección IPv6 del siguiente salto, o `::/128`.
     * si no se encuentra ninguno.
     */
    const inet::Ipv6Address& findNextHopClosestToVertex(Vertex vertex) const;
    /*!
     * @brief Agrega los siguientes vértices visitados a la ruta.
     *
     * @param route        [in] Ruta a la que se le agregan los datos de ruta.
     * @param shortestPath [in] Ruta vial.
     */
    void setNewRouteData(inet::Ipv6Route *route,
            const VertexVector &shortestPath) const;
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
