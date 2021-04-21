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
 * @file RoutingProtocolBase.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include <omnetpp.h>
#include "inet/common/INETUtils.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/Ptr.h"
#include "inet/common/TlvOptions_m.h"
#include "inet/networklayer/common/NetworkInterface.h"
#include "inet/networklayer/common/InterfaceTable.h"
#include "inet/networklayer/common/L3Tools.h"
#include "inet/networklayer/contract/INetfilter.h"
#include "inet/networklayer/contract/ipv6/Ipv6Address.h"
#include "inet/networklayer/ipv6/Ipv6ExtensionHeaders_m.h"
#include "inet/networklayer/ipv6/Ipv6Header_m.h"
#include "inet/networklayer/ipv6/Ipv6RoutingTable.h"
#include "inet/routing/base/RoutingProtocolBase.h"
#include "veins_proj/routing/Routing_m.h"
#include "veins_proj/roadnetwork/RoadNetworkDatabase.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "veins_proj/util/ExpiringValuesMap.h"
#include <string>
#include <map>
#include <utility>

#define ROUTING_PROTOCOL_UDP_PORT                       4096
#define IPV6TLVOPTION_TLV_DEST_GEOHASH_LOCATION         75
#define IPV6TLVOPTION_TLV_DEST_ON_ROAD_NETWORK_LOCATION 76
#define IPV6TLVOPTION_TLV_ROUTING                       77

namespace veins_proj {

/*!
 * @brief Módulo que implementa las operaciones de enrutamiento comunes para
 * vehículos y _hosts_.
 *
 * Contiene los parámetros de enrutamiento para configurar el protocolo.
 * Las unidades de los parámetros se especifican en el archivo de configuración.
 */
class RoutingProtocolBase: public inet::RoutingProtocolBase,
                           public omnetpp::cListener,
                           public inet::NetfilterBase::HookBase {

protected:

    /*
     * Parámetros de configuración.
     */
    //! Hora de inicio.
    omnetpp::simtime_t startTime;
    //! Intervalo de transmisión de mensajes HOLA_VEHIC.
    omnetpp::simtime_t helloVehicleInterval;
    //! Tiempo de vigencia de los registros del de vehículos vecinos.
    omnetpp::simtime_t neighbouringVehicleValidityTime;
    //! Intervalo de transmisión de mensajes HOLA_HOST.
    omnetpp::simtime_t helloHostInterval;
    //! Tiempo de vigencia de los registros del directorio de *hosts* vecinos.
    omnetpp::simtime_t neighbouringHostValidityTime;
    //! Tiempo de vigencia de las rutas.
    omnetpp::simtime_t routeValidityTime;
    //! Tiempo de demora para transmisión de paquetes UDP consecutivos.
    omnetpp::simtime_t udpPacketDelayTime;

    /*
     * Contexto.
     */
    //! Módulo del vehículo o *host* o vehículo contenedor.
    omnetpp::cModule *host = nullptr;
    //! Módulo de la tabla de interfaces.
    inet::IInterfaceTable *interfaceTable = nullptr;
    //! Módulo de la interfaz de red.
    inet::NetworkInterface *networkInterface = nullptr;
    //! Módulo de la tabla de enrutamiento IPv6.
    inet::Ipv6RoutingTable *routingTable = nullptr;
    //! Módulo del protocolo de red.
    inet::INetfilter *ipv6Protocol = nullptr;
    //! Módulo de la base de datos de redes viales.
    RoadNetworkDatabase *roadNetworkDatabase = nullptr;

    /*
     * Mensajes propios.
     */
    //! Temporizador de limpieza del directorio de vehículos vecinos.
    omnetpp::cMessage *purgeNeighbouringVehiclesTimer = nullptr;

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
    virtual void handleMessageWhenUp(omnetpp::cMessage *message) override;

    /*
     * Manejo de mensajes.
     */
    /*!
     * @brief Manejo de mensajes.
     *
     * @param message [in] Mensaje a procesar.
     */
    virtual void processMessage(omnetpp::cMessage *message);
    /*!
     * @brief Manejo de mensajes propios.
     *
     * @param message [in] Mensaje a procesar.
     */
    virtual void processSelfMessage(omnetpp::cMessage *message);

    /*
     * Paquetes UDP.
     */
    /*!
     * @brief Enviar paquete UDP.
     *
     * Envía un paquete UDP hacia la compuerta que se conecta con el
     * protocolo IP.
     *
     * @param packet  [in] Paque a enviar.
     * @param delayed [in] Indica si la transmisión del paquete se va a demorar.
     * Se utiliza si se tienen que enviar dos paquetes consecutivamente.
     */
    void sendUdpPacket(inet::Packet *packet, bool delayed = false);
    /*!
     * @brief Procesar paquete UDP.
     *
     * Identifica el tipo de paquete y lo envía a la función de procesamiento
     * correspondiente.
     *
     * @param packet [in] Paquete a procesar.
     */
    void processUdpPacket(inet::Packet *packet);

    /*
     * Mensajes de enrutamiento.
     */
    /*!
     * @brief Se encapsula el mensaje de enrutamiento en un datagrama UDP
     * y se envía a la dirección indicada.
     *
     * @param routingMessage [in] Mensaje a enviar.
     * @param name           [in] Nombre del mensaje.
     * @param srcAddress     [in] Dirección de origen del mensaje.
     * @param destAddress    [in] Dirección de destino del mensaje.
     * @param delayed        [in] Indica si la transmisión del paquete se va
     *                            a demorar. Se utiliza si se tienen que enviar
     *                            dos mensajes consecutivamente.
     */
    void sendRoutingMessage(const inet::Ptr<RoutingPacket> routingMessage,
            const char *name, const inet::Ipv6Address &srcAddress,
            const inet::Ipv6Address &destAddress, bool delayed = false);

    /*
     * Mensajes HOLA_VEHIC.
     */
    /*!
     * @brief Procesar mensaje HOLA_VEHIC.
     *
     * @param helloVehicle [in] Mensaje a procesar.
     */
    virtual void processHelloVehicle(
            const inet::Ptr<HelloVehicle> &helloVehicle) {
    }

    /*
     * Mensajes HOLA_HOST.
     */
    /*!
     * @brief Procesar mensaje HOLA_HOST.
     *
     * @param helloHost [in] Mensaje a procesar.
     */
    virtual void processHelloHost(const inet::Ptr<HelloHost> &helloHost) {
    }

    /*
     * Directorio de vehículos vecinos.
     */
    //! Registro de vehículo vecino.
    struct NeighbouringVehicleValue {
        //! Ubicación Geohash del vehículo.
        GeohashLocation geohashLocation;
        //! Velocidad de movimiento del vehículo en metros por segundo.
        double speed;
        //! Dirección acimutal del movimiento del vehículo en grados.
        double direction;
        //! Ubicación vial del vehículo.
        LocationOnRoadNetwork locationOnRoadNetwork;
    };
    //! Diccionario de directorio de vehículos vecinos.
    typedef ExpiringValuesMap<inet::Ipv6Address, NeighbouringVehicleValue> NeighbouringVehicles;
    //! Valor en el directorio de vehículos vecinos.
    typedef NeighbouringVehicles::MapValue NeighbouringVehicle;
    //! Iterador de registros para diccionario del directorio
    //! de vehículos vecinos.
    typedef NeighbouringVehicles::It NeighbouringVehiclesIt;
    //! Iterador de registros para diccionario de directorio
    //! de vehículos vecinos constante.
    typedef NeighbouringVehicles::ConstIt NeighbouringVehiclesConstIt;
    //! Directorio de vehículos vecinos.
    NeighbouringVehicles neighbouringVehicles;
    /*!
     * @brief Imprimir el directorio de vehículos vecinos.
     */
    virtual void showNeighbouringVehicles() const;
    /*!
     * @brief Programar el temporizador de limpieza del directorio de
     * vehículos vecinos.
     */
    virtual void schedulePurgeNeighbouringVehiclesTimer();
    /*!
     * @brief Procesar el temporizador de limpieza del directorio de
     * vehículos vecinos.
     */
    virtual void processPurgeNeighbouringVehiclesTimer();
    /*!
     * @brief Obtener el vehículo vecino más cercano.
     *
     * Encuentra el vehículo vecino cuya ubicación es la más cercana
     * a la ubicación indicada.
     *
     * @param geohashLocation [in] Ubicación Geohash de referencia.
     *
     * @return Dirección IPv6 del vehículo vecino más cercano.
     */
    inet::Ipv6Address findClosestNeighbouringVehicle(
            const GeohashLocation &geohashLocation) const;

    /*
     * Rutas.
     */
    /*!
     * @brief Mostrar rutas en la tabla de enrutamiento.
     */
    void showRoutes() const;
    /*!
     * @brief Eliminar rutas expiradas de la tabla de enrutamiento.
     *
     * @param expiryTime [in] Hora de expiración máxima para eliminar las rutas.
     */
    void removeExpiredRoutes(omnetpp::simtime_t expiryTime);

    /*
     * Enrutamiento
     */
    /*!
     * @brief Enrutar datagrama.
     *
     * Revisa si existe en la tabla de enrutamiento una ruta hacia la
     * dirección de destino. Si no existe, se intenta descubrir y crear una
     * ruta. Si no se encuentra la ruta, se descarta el datagrama.
     *
     * @param datagram [in] Datagrama a enrutar.
     *
     * @return Resultado del enrutamiento.
     */
    virtual inet::INetfilter::IHook::Result routeDatagram(
            inet::Packet *datagram) = 0;

    /*
     * Cabecera de opciones de salto por salto.
     */
    /*!
     * @brief Agregar opción TLV a un datagrama.
     *
     * @param datagram [in] Datagrama al que se le agregará la opción TLV.
     * @param tlvOption [in] Opción TLV a agregar al datagrama.
     */
    template<class T> void setTlvOption(inet::Packet *datagram,
            T *tlvOption) const {

        /*
         * Se obtiene la cabecera de opciones de salto por salto
         * de la cabecera IPv6.
         */
        datagram->trimFront();

        inet::Ptr<inet::Ipv6Header> ipv6Header =
                inet::removeNetworkProtocolHeader<inet::Ipv6Header>(datagram);
        inet::B oldHeaderLength = ipv6Header->calculateHeaderByteLength();
        inet::Ipv6ExtensionHeader *extensionHeader =
                ipv6Header->findExtensionHeaderByTypeForUpdate(
                        inet::IpProtocolId::IP_PROT_IPv6EXT_HOP);
        inet::Ipv6HopByHopOptionsHeader *optionsHeader =
                omnetpp::check_and_cast_nullable<
                        inet::Ipv6HopByHopOptionsHeader*>(extensionHeader);
        /*
         * Si no existe la cabecera de opciones de salto por salto,
         * se agrega una nueva.
         */
        if (!optionsHeader) {
            optionsHeader = new inet::Ipv6HopByHopOptionsHeader();
            optionsHeader->setByteLength(inet::B(8));
            ipv6Header->addExtensionHeader(optionsHeader);
        }
        /*
         * Se verifica si ya existe una opción TLV en la cabecera
         * de la misma clase de la opción TLV que se quiere agregar.
         */
        inet::TlvOptions &tlvOptions = optionsHeader->getTlvOptionsForUpdate();
        size_t i = 0;
        size_t n = tlvOptions.getTlvOptionArraySize();
        while (i < n) {
            if (dynamic_cast<T*>(tlvOptions.getTlvOptionForUpdate(i)))
                break;
            i++;
        }
        /*
         * Si se encontró la opción de la misma clase,
         * se elimina esta y se cambia por la nueva.
         */
        if (i < n) {
            delete tlvOptions.dropTlvOption(i);
            tlvOptions.setTlvOption(i, tlvOption);
            /*
             * Si no se encontró la opción de la misma clase,
             * únicamente se agrega la nueva.
             */
        } else
            tlvOptions.insertTlvOption(tlvOption);
        /*
         * Se actualiza la longitud de la cabecera y se reinserta en el datagrama.
         */
        optionsHeader->setByteLength(
                inet::B(
                        inet::utils::roundUp(
                                2 + inet::B(tlvOptions.getLength()).get(), 8)));
        inet::B newHeaderLength = ipv6Header->calculateHeaderByteLength();
        ipv6Header->addChunkLength(newHeaderLength - oldHeaderLength);
        inet::insertNetworkProtocolHeader(datagram, inet::Protocol::ipv6,
                ipv6Header);
    }
    /*!
     * @brief Obtener opción TLV constante de la cabecera de opciones
     * de salto por salto de un datagrama.
     *
     * @tparam T Tipo de opción TLV a obtener.
     * @param datagram [in] Datagrama del que se obtiene la opción TLV.
     *
     * @return Opción TLV.
     */
    template<class T> const T* findTlvOption(inet::Packet *datagram) const {
        /*
         * Se obtiene la cabecera de opciones de salto por salto
         * de la cabecera IPv6.
         */
        inet::Ptr<const inet::Ipv6Header> ipv6Header = inet::dynamicPtrCast<
                const inet::Ipv6Header>(
                inet::getNetworkProtocolHeader(datagram));
        const inet::Ipv6ExtensionHeader *extensionHeader =
                ipv6Header->findExtensionHeaderByType(
                        inet::IpProtocolId::IP_PROT_IPv6EXT_HOP);
        const inet::Ipv6HopByHopOptionsHeader *optionsHeader =
                omnetpp::check_and_cast_nullable<
                        const inet::Ipv6HopByHopOptionsHeader*>(
                        extensionHeader);
        /*
         * Si se encuentra la cabecera de opciones de salto por salto,
         * se busca la opción TLV de la clase `T`.
         */
        const T *tlvOption = nullptr;
        if (optionsHeader) {
            const inet::TlvOptions &tlvOptions = optionsHeader->getTlvOptions();
            size_t i = 0;
            while (i < tlvOptions.getTlvOptionArraySize()) {
                tlvOption =
                        dynamic_cast<const T*>(tlvOptions.getTlvOption(i++));
                if (tlvOption)
                    break;
            }
        }

        return tlvOption;
    }
    /*!
     * @brief Eliminar una opción TLV de la cabecera de opciones
     * de salto por salto de un datagrama.
     *
     * @tparam T Tipo de opción TLV a eliminar.
     * @param datagram [in] Datagrama del que se elimina la opción TLV.
     */
    template<class T> void removeTlvOption(inet::Packet *datagram) const {
        /*
         * Se obtiene la cabecera de opciones de salto por salto
         * de la cabecera IPv6.
         */
        inet::Ptr<inet::Ipv6Header> ipv6Header = inet::constPtrCast<
                inet::Ipv6Header>(
                inet::dynamicPtrCast<const inet::Ipv6Header>(
                        inet::getNetworkProtocolHeader(datagram)));
        inet::Ipv6ExtensionHeader *extensionHeader =
                ipv6Header->findExtensionHeaderByTypeForUpdate(
                        inet::IpProtocolId::IP_PROT_IPv6EXT_HOP);
        inet::Ipv6HopByHopOptionsHeader *optionsHeader =
                omnetpp::check_and_cast_nullable<
                        inet::Ipv6HopByHopOptionsHeader*>(extensionHeader);
        /*
         * Si se encuentra la cabecera de opciones de salto por salto,
         * se busca la opción TLV de la clase `T`.
         */
        if (optionsHeader) {
            inet::TlvOptions &tlvOptions =
                    optionsHeader->getTlvOptionsForUpdate();
            int i = 0;
            while (i < tlvOptions.getTlvOptionArraySize()) {
                if (dynamic_cast<T*>(tlvOptions.getTlvOptionForUpdate(i))) {
                    tlvOptions.eraseTlvOption(i);
                    break;
                }
                i++;
            }
        }
    }
    /*!
     * @brief Crear opción TLV de ubicación del destino.
     *
     * @param geohashLocationBits [in] Ubicación Geohash del destino.
     *
     * @return Opción TLV de ubicación del destino.
     */
    TlvDestGeohashLocationOption* createTlvDestGeohashLocationOption(
            uint64_t geohashLocationBits) const;
    /*!
     * @brief Calcular la longitud en octetos de una opción TLV de ubicación
     * del destino.
     *
     * @param tlvOption [in] Opción TLV cuya longitud se calcula.
     *
     * @return Longitud de la opción TLV.
     */
    int computeTlvOptionLength(TlvDestGeohashLocationOption *tlvOption) const;
    /*!
     * @brief Crear opción TLV de ubicación vial del destino.
     *
     * @param vertexA [in] Primer vértice de la arista de la ubicación vial.
     * @param vertexB [in] Segundo vértice de la arista de la ubicación vial.
     * @param distanceToVertexA [in] Distancia al primer vértice.
     *
     * @return Opción TLV de ubicación vial del destino.
     */
    TlvDestLocationOnRoadNetworkOption* createTlvDestLocationOnRoadNetworkOption(
            const GeohashLocation &destGeohashLocation) const;
    /*!
     * @brief Calcular la longitud en octetos de una opción TLV de
     * ubicación vial del destino.
     *
     * @param tlvOption [in] Opción TLV cuya longitud se calcula.
     *
     * @return Longitud de la opción TLV.
     */
    int computeTlvOptionLength(
            TlvDestLocationOnRoadNetworkOption *tlvOption) const;
    /*!
     * @brief Crear opción TLV de vértices visitados vacía.
     *
     *@param visitedVertices [in] Conjunto de vértices visitados.
     * @return Opción TLV de vértices visitados.
     */
    TlvVisitedVerticesOption* createTlvVisitedVerticesOption(
            const VertexSet &visitedVertices = { }) const;
    /*!
     * @brief Calcular la longitud en octetos de una opción TLV de
     * vértices visitados.
     *
     * @param tlvOption [in] Opción TLV cuya longitud se calcula.
     *
     * @return Longitud de la opción TLV.
     */
    int computeTlvOptionLength(TlvVisitedVerticesOption *tlvOption) const;

    /*
     * Netfilter.
     */
    /*!
     * @brief Procesar datagrama recibido de la capa inferior
     * antes de enrutarlo.
     *
     * @param datagram [in] Datagrama a procesar.
     *
     * @return Resultado del procesamiento.
     */
    virtual inet::INetfilter::IHook::Result datagramPreRoutingHook(
            inet::Packet *datagram) override {
        return inet::INetfilter::IHook::ACCEPT;
    }
    virtual inet::INetfilter::IHook::Result datagramForwardHook(
            inet::Packet *datagram) override {
        return inet::INetfilter::IHook::ACCEPT;
    }
    virtual inet::INetfilter::IHook::Result datagramPostRoutingHook(
            inet::Packet *datagram) override {
        return inet::INetfilter::IHook::ACCEPT;
    }
    virtual inet::INetfilter::IHook::Result datagramLocalInHook(
            inet::Packet *datagram) override {
        return inet::INetfilter::IHook::ACCEPT;
    }
    /*!
     * @brief Procesar datagrama recibido de la capa superior
     * antes de enrutarlo.
     *
     * @param datagram [in] Datagrama a procesar.
     *
     * @return Resultado del procesamiento.
     */
    virtual inet::INetfilter::IHook::Result datagramLocalOutHook(
            inet::Packet *datagram) override {
        return inet::INetfilter::IHook::ACCEPT;
    }

    /*
     * Lifecycle.
     */
    virtual void handleStartOperation(inet::LifecycleOperation *operation)
            override = 0;
    virtual void handleStopOperation(inet::LifecycleOperation *operation)
            override;
    virtual void handleCrashOperation(inet::LifecycleOperation *operation)
            override;

    /*
     * Notification.
     */
    virtual void receiveSignal(omnetpp::cComponent *source,
            omnetpp::simsignal_t signalID, omnetpp::cObject *obj,
            cObject *details) override = 0;
};

}    // namespace veins_proj
