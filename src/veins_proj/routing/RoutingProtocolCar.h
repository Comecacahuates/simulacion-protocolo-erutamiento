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

#include "veins_proj/networklayer/configurator/AddressCache.h"
#include <omnetpp.h>
#include "inet/common/packet/Packet.h"
#include "inet/common/Ptr.h"
#include "inet/networklayer/common/InterfaceTable.h"
#include "inet/networklayer/contract/ipv6/Ipv6Address.h"
#include "inet/networklayer/contract/INetfilter.h"
#include "inet/networklayer/ipv6/Ipv6RoutingTable.h"
#include "veins_proj/routing/RoutingProtocolBase.h"
#include "veins_proj/routing/Routing_m.h"
#include "veins_proj/locationservice/HostsLocationTable.h"
#include "veins_proj/mobility/CarMobility.h"
#include "veins_proj/roadnetwork/RoadNetworkDatabase.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "veins_proj/roadnetwork/ShortestPath.h"
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
    /*!
     * @brief Módulo de movilidad.
     */
    CarMobility *mobility = nullptr;

    /*
     * Vehículos vecinos por arista.
     * TODO Revisar si hace falta.
     */
    typedef std::pair<Edge, inet::Ipv6Address> NeighbouringCarByEdge;
    typedef std::multimap<Edge, inet::Ipv6Address> NeighbouringCarsByEdgeMap;
    typedef NeighbouringCarsByEdgeMap::iterator NeighbouringCarsByEdgeIterator;
    typedef NeighbouringCarsByEdgeMap::const_iterator NeighbouringCarsByEdgeConstIterator;
    NeighbouringCarsByEdgeMap neighbouringCarsByEdge;

    /*
     * Mensajes propios.
     */
    /*!
     * @brief Temporizador de transmisión de mensajes HOLA_VEHIC.
     */
    omnetpp::cMessage *helloCarTimer;
    /*!
     * @brief Temporizador de limpieza del directorio de _hosts_ vecinos.
     */
    omnetpp::cMessage *purgeNeighbouringHostsTimer;
    /*!
     * @brief Temporizador de limpieza de aristas activas.
     */
    omnetpp::cMessage *purgeEdgesStatusTimer;
    /*!
     * @brief Temporizador de datagramas demorados.
     */
    omnetpp::cMessage *purgeDelayedDatagramsTimer;
    /*!
     * @brief Temporizador de limpieza de mensajes PONG pendientes.
     */
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
     *
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
     * @param carAddress [in] Dirección del vehículo remitente.
     * @param pingVertex [in] Vértice de origen.
     * @param pongVertex [in] Vértice de destino.
     *
     * @return Mensaje PING.
     */
    virtual const inet::Ptr<Ping> createPing(
            const inet::Ipv6Address &carAddress, Vertex pingVertex,
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
     *
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
     * Directorio de vehículos vecinos.
     */
    /*!
     * @brief Obtener vehículo vecino aleatorio en la misma arista.
     *
     * Obtiene aleatoriamente un vehículo vecino que se encuentra en la misma
     * arista, y que esté más cerca del vértice indicado.
     *
     * @param targetVertex [in] Vértice de referencia.
     *
     * @return Dirección IPv6 del vehículo vecino seleccionado.
     */
    virtual inet::Ipv6Address getRandomNeighbouringCarAddressAheadOnEdge(
            Vertex targetVertex) const;
    /*!
     * @brief Buscar vehículo vecino más cercano a un vértice que se encuentra en
     * la misma arista.
     *
     * Se bucan los vehículos vecinos que circulan sobre la misma arista,
     * y se obtiene el que se encuentra a la menor distancia del vértice
     * indicado.
     *
     * @param vertex [in] Vértice de referencia.
     *
     * @return Dirección IPv6 del vecino encontrado, o `::/128`
     * si no se encuentra ninguno.
     */
    virtual inet::Ipv6Address findNeighbouringCarClosestToVertex(Vertex vertex);
    /*!
     * @brief Obtener la cantidad de vehículos vecinos que se encuentran
     * en la misma arista.
     *
     * @return Cantidad de vehículos vecinos que se encuentran en
     * la misma arista.
     */
    virtual int getNeighbouringCarsOnEdgeCount() const;

    /*
     * Directorio de hosts vecinos.
     */
    /*!
     * @brief Diccionario de directorio de _hosts_ vecinos.
     *
     * La clave es la dirección IPv6 del _host_ vecino, y el valor es el
     * registro de _host_ vecino.
     */
    typedef ExpiringValuesMap<inet::Ipv6Address, GeohashLocation> NeighbouringHosts;
    /*!
     * @brief Registro de diccionario de directorio de _hosts_ vecinos.
     *
     * La clave es la dirección IPv6 del _host_ vecino, y el valor es el
     * registro de _host_ vecino.
     */
    typedef std::pair<inet::Ipv6Address, GeohashLocation> NeighbouringHost;
    /*!
     * @brief Iterador de registros para diccionario del directorio
     * de _hosts_ vecinos.
     */
    typedef NeighbouringHosts::Iterator NeighbouringHostsIterator;
    /*!
     * @brief Iterador de registros para diccionario del directorio de _hosts_
     * vecinos constante.
     */
    typedef NeighbouringHosts::ConstIterator NeighbouringHostsConstIterator;
    /*!
     * @brief Directorio de _hosts_ vecinos.
     */
    NeighbouringHosts neighbouringHosts;
    /*!
     * @brief Imrpimir el directorio de _hosts_ vecinos.
     */
    virtual void showNeighbouringHosts() const;
    /*!
     * @brief Programar el temporizador de limpieza del directorio
     * de _hosts_ vecinos.
     */
    virtual void schedulePurgeNeighbouringHostsTimer();
    /*!
     * @brief Procesar el temporizador de limpieza del directorio
     * de _hosts_ vecinos.
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
    typedef ExpiringValuesMultimap<inet::Ipv6Address, inet::Packet*> DelayedDatagrams;
    /*!
     * @brief Valor.
     */
    typedef DelayedDatagrams::MultimapValue DelayedPacket;
    /*!
     * @brief Iterador para diccionario de datagramas demorados.
     */
    typedef DelayedDatagrams::Iterator DelayedDatagramsIterator;
    /*!
     * @brief Iterador para diccionario de datagramas demorados constante.
     */
    typedef DelayedDatagrams::ConstIterator DelayedDatagramsConstIterator;
    /*!
     * @brief Paquetes demorados.
     */
    DelayedDatagrams delayedDatagrams;
    /*!
     * @brief Imprimir los datagramas demorados.
     */
    virtual void showDelayedDatagrams();
    /*!
     * @brief Programar el temporizador de limpieza de datagramas demorados.
     */
    virtual void schedulePurgeDelayedDatagramsTimer();
    /*!
     * @brief Procesar el temporizador de limpieza de datagramas demorados.
     */
    virtual void processPurgeDelayedDatagramsTimer();

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
        Vertex srcVertex;
        /*!
         * @brief Vértice de destino.
         */
        Vertex destVertex;
    };
    /*!
     * @brief Diccionario de mensajes PONG pendientes.
     *
     * La clave es la arista por la que se envió el mensaje PING,
     * y el valor es la hora de expiración.
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
    virtual void showPendingPongs() const;
    /*!
     * @brief Programar el temporizador de limpieza de mensajes
     * PONG pendientes.
     */
    virtual void schedulePurgePendingPongsTimer();
    /*!
     * @brief Procesar el temporizador de limpieza de mensajes
     * PONG pendientes.
     */
    virtual void processPurgePendingPongsTimer();

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
     * @param destAddress [in] Dirección IPv6 de destino.
     *
     * @return Resultado del enrutamiento.
     */
    inet::INetfilter::IHook::Result routeDatagram(inet::Packet *datagram,
            const inet::Ipv6Address &destAddress) override;
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
     * @brief Obtener el vértice de destino local.
     *
     * @param datagram [in] Datagrama a enrutar.
     * @param shortestPath [in] Rutas más cortas.
     *
     * @return Vértice de destino local.
     */
    Vertex getLocalDestVertex(inet::Packet *datagram,
            const ShortestPath &shortestPath) const;
    /*!
     * @brief Se obtiene el conjunto de vértices visitados.
     *
     * @param visitedVerticesOption [in] Opción de vértices visitados.
     *
     * @return Conjunto de vértices visitados.
     */
    VertexSet getVisitedVertices(
            TlvVisitedVerticesOption *visitedVerticesOption) const;
    /*!
     * @brief Obtener el vértice de destino.
     *
     * @param destGeohashLocation [in] Ubicación Geohash del destino.
     * @param destEdge [in] Arista de la ubicación del destno.
     * @param shortestPath [in] Rutas más cortas.
     *
     * @return Vértice de destino.
     */
    Vertex getDestVertex(const GeohashLocation &destGeohashLocation,
            Edge destEdge, const ShortestPath &shortestPath) const;
    /*!
     * @brief Obtener aristas en la ruta más corta que forman un tramo recto.
     *
     * Se obtienen las aristas en la ruta que forman el tramo largo más recto,
     * y en las que haya vehículos vecinos circulando.
     *
     * @param shortestPathToDestVertex [in] Ruta más corta al vértice
     * de destino.
     * @param shortestPath [in] Rutas más cortas.
     *
     * @return Aristas que forman un
     */
    EdgeVector getReachableEdges(const VertexVector &shortestPathToDestVertex,
            const ShortestPath &shortestPath) const;
    /*!
     * @brief Encontrar siguiente salto.
     *
     * Se obtiene el siguiente salto en la ruta.
     *
     * @param shortestPathToDestVertex [in] Ruta más corta al vértice
     * de destino.
     * @param shortestPath [in] Rutas más cortas.
     *
     * @return Dirección IPv6 del siguiente salto.
     */
    inet::Ipv6Address findNextHop(const VertexVector &shortestPathToDestVertex,
            const ShortestPath &shortestPath) const;
    /*!
     * @brief Obtener vehículo vecino en la región Geohash adyacente.
     *
     * @param neighbouringGeohashRegion [in] Región Geohash adyacente.
     *
     * @return Dirección IPv6 del vehículo vecino en la región Geohash indicada.
     */
    inet::Ipv6Address findNeighbourInNeighbourinRegion(
            const GeohashLocation &neighbouringGeohashRegion) const;

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
    virtual inet::INetfilter::IHook::Result datagramPreRoutingHook(
            inet::Packet *datagram) override;
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
