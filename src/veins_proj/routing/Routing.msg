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


enum PacketType {
    ACK = 0;
    HELLO_CAR = 1;
    HELLO_HOST = 2;
    PING = 3;
    PONG = 4;
    EDGE_STATUS = 5;
};

class RoutingPacket extends inet::FieldsChunk {
	PacketType packetType;
}

class Ack extends RoutingPacket {
    chunkLength = inet::B(1 + 16);
    packetType = PacketType::ACK;
	inet::Ipv6Address address;
}

class HelloCar extends RoutingPacket {
    chunkLength = inet::B(40);
    packetType = PacketType::HELLO_CAR;
	inet::Ipv6Address srcAddress; // Dirección IPv6 (16 bytes)
	uint64_t geohash; // Ubicación Geohash (8 bytes)
	double speed; // Velocidad (2 bytes)
	double direction; // Dirección de movimiento (2 bytes)
	unsigned int vertexA; // Vértice A (2 bytes)
	unsigned int vertexB; // Vértice B (2 bytes)
	double distanceToVertexA; // Distancia al Vértice A (2 bytes)
	bool pingPong = false; // Bandera ping-pong (1 bit)
	bool pingPongError = false; // Bandera de error ping-pong (1 bit)
	unsigned int pingVertex = 0; // Vértice de origen del mensaje PING (2 bytes)
	unsigned int pongVertex = 0; // Vértice de destino del mensaje PONG (2 bytes)
}

class HelloHost extends RoutingPacket {
    chunkLength = inet::B(28);
    packetType = PacketType::HELLO_HOST;
	inet::Ipv6Address address; // Dirección IPv6 de origen (16 bytes)
	uint64_t geohash; // Ubicación Geohash (8 bytes)
}

class Ping extends RoutingPacket {
    chunkLength = inet::B(40);
    packetType = PacketType::PING;
	inet::Ipv6Address address; // Dirección IPv6 de origen (16 bytes)
	unsigned int pingVertex; // Vértice de origen (2 bytes)
	unsigned int pongVertex; // Vértice de destino (2 bytes)
}

class Pong extends RoutingPacket {
    chunkLength = inet::B(24);
    packetType = PacketType::PONG;
    bool error; // Error en la operación ping-pong (1 bit)
	inet::Ipv6Address destAddress; // Dirección IPv6 (16 bytes)
	unsigned int pingVertex; // Vértice de origen (2 bytes)
	unsigned int pongVertex; // Vértice de destino (2 bytes)
}

class HelloEdge extends RoutingPacket {
	chunkLength = inet::B(8);
    packetType = PacketType::EDGE_STATUS;
    bool isActive; // Indica si la ruta está activa (1 bit)
	int hopCount; // Número de saltos (1 byte)
	int neighboursAverage; // Promedio de vehículos vecinos (1 byte)
	unsigned int vertexA; // Vértice A (2 bytes)
	unsigned int vertexB; // Vértice B (2 bytes)
}

class TlvDestGeohashLocationOption extends inet::TlvOptionBase {
	uint64_t geohash; // Ubicaci��n Geohash (8 bytes)
}

class TlvDestLocationOnRoadNetworkOption extends inet::TlvOptionBase {
	unsigned int vertexA; // Vértice A (2 bytes)
	unsigned int vertexB; // Vértice B (2 bytes)
	double distanceToVertexA; // Distancia al Vértice A (2 bytes)
}

class TlvVisitedVerticesOption extends inet::TlvOptionBase {
	unsigned int visitedVertices[]; // Vértices que ha recorrido el paquete (2 bytes por Vértice)
}
