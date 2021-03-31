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
    HELLO_CAR = 0;
    HELLO_HOST = 1;
};

class RoutingPacket extends inet::FieldsChunk {
	PacketType packetType;
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
}

class HelloHost extends RoutingPacket {
    chunkLength = inet::B(28);
    packetType = PacketType::HELLO_HOST;
	inet::Ipv6Address address; // Dirección IPv6 de origen (16 bytes)
	uint64_t geohash; // Ubicación Geohash (8 bytes)
}


class TlvDestGeohashLocationOption extends inet::TlvOptionBase {
	uint64_t geohash; // Ubicación Geohash (8 bytes)
}

class TlvDestLocationOnRoadNetworkOption extends inet::TlvOptionBase {
	unsigned int vertexA; // Vértice A (2 bytes)
	unsigned int vertexB; // Vértice B (2 bytes)
	double distanceToVertexA; // Distancia al Vértice A (2 bytes)
}

class TlvVisitedVerticesOption extends inet::TlvOptionBase {
	unsigned int visitedVertices[]; // Vértices que ha recorrido el paquete (2 bytes por Vértice)
}
