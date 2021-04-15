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

import inet.common.INETDefs;
import inet.common.TlvOptions;
import inet.common.packet.chunk.Chunk;
import inet.networklayer.contract.ipv6.Ipv6Address;

namespace veins_proj;

//
// Tipos de mensajes.
//
// @author Adrián Juárez
//
enum PacketType {
    // Tipo de mensajes HOLA-VEHIC
    HELLO_VEHIC = 0;
    // Tipo de mensajes HOLA-HOST
    HELLO_HOST = 1;
};

//
// Clase abstracta base para los mensajes de enrutamiento.
//
// @author Adrián Juárez
//
class RoutingPacket extends inet::FieldsChunk {
    // Tipo de mensaje
	PacketType packetType;
}

//
// Mensaje HOLA-VEHIC.
//
// @author Adrián Juárez
//
class HelloVehicle extends RoutingPacket {
    chunkLength = inet::B(40);
    packetType = PacketType::HELLO_VEHIC;
    // Dirección IPv6 (16 bytes)
	inet::Ipv6Address srcAddress;
	// Ubicación Geohash (8 bytes)
	uint64_t geohash;
	// Velocidad (2 bytes)
	double speed;
	// Dirección de movimiento (2 bytes)
	double direction;
	// Vértice A (2 bytes)
	unsigned int vertexA;
	// Vértice B (2 bytes)
	unsigned int vertexB;
	// Distancia al Vértice A (2 bytes)
	double distanceToVertexA;
}

//
// Mensaje HOLA-HOST.
//
// @author Adrián Juárez
//
class HelloHost extends RoutingPacket {
    chunkLength = inet::B(28);
    packetType = PacketType::HELLO_HOST;
    // Dirección IPv6 de origen (16 bytes)
	inet::Ipv6Address address;
	// Ubicación Geohash (8 bytes)
	uint64_t geohash;
}

//
// Opción de ubicación del destino.
//
// @author Adrián Juárez
//
class TlvDestGeohashLocationOption extends inet::TlvOptionBase {
    // Ubicación Geohash (8 bytes)
	uint64_t geohash;
}

//
// Opción de ubicación vial del destino.
//
// @author Adrián Juárez
//
class TlvDestLocationOnRoadNetworkOption extends inet::TlvOptionBase {
	// Vértice A (2 bytes)
	unsigned int vertexA;
	// Vértice B (2 bytes)
	unsigned int vertexB;
	// Distancia al Vértice A (2 bytes)
	double distanceToVertexA;
}

//
// Opción de vértices visitados.
//
// @author Adrián Juárez
//
class TlvVisitedVerticesOption extends inet::TlvOptionBase {
    // Vértices que ha recorrido el paquete (2 bytes por Vértice)
	unsigned int visitedVertices[];
}
