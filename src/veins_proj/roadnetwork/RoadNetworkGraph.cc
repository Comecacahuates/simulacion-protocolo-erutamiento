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
 * @file RoadNetworkGraph.cc
 * @author Adrián Juárez Monroy
 */

#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "boost/swap.hpp"
#include <utility>
#include <limits>

using namespace veins_proj;

//! Determina si la ubicación del primer vértice es menor a la del segundo.
/*!
 * Para que la ubicación del primer vértice sea menor, su latitud debe ser
 * menor a la del segundo. Si las latitudes son iguales, la longitud debe ser
 * menor.
 *
 * @param vertexA [in] Primer vértice cuya ubicación se va a comparar.
 * @param vertexB [in] Segundo vértice cuya ubicación se va a comparar.
 * @param graph [in] Grafo al que pertenecen los dos vértices.
 * @return `true` si la ubicación del primer vértice es menor a la del
 * segundo.
 */
bool veins_proj::sortedVertices(Vertex vertexA, Vertex vertexB,
		const Graph &graph) {
	const GeographicLib::GeoCoords &locationA = graph[vertexA].location;
	const GeographicLib::GeoCoords &locationB = graph[vertexB].location;

	if (locationA.Latitude() > locationB.Latitude())
		return false;

	else if (locationA.Latitude() == locationB.Latitude())
		if (locationA.Longitude() > locationB.Longitude())
			return false;

	return true;
}

//! Determina si un vértice es _gateway_.
/*!
 * @param vertex [in] Vértice que se va a revisar.
 * @param graph [in] Grafo al que pertenece el vértice.
 * @return `true` en caso de que el vértice sea _gateway_.
 */
bool veins_proj::isGateway(Vertex vertex, const Graph &graph) {
	return graph[vertex].adjacency != GeohashLocation::Adjacency::NONE;
}

//! Verifica si dos direcciones acimutales coinciden.
/*!
 * Para que dos direcciones coincidan, su diferencia debe ser menor
 * a 15 grados.
 *
 * @param direction1 [in] Primera dirección en grados.
 * @param direction2 [in] Segunda dirección en grados.
 * @return `true` si las dos direcciones coinciden.
 */
bool veins_proj::directionMatches(double direction1, double direction2) {
	return getDirectionDifference(direction1, direction2) < 15.0;
}

//! Calcula la diferencia de dos direcciones acimutales.
/*!
 * La diferencia es el ángulo menor, en grados, que forman dos direcciones.
 *
 * @param direction1 [in] Primera dirección en grados.
 * @param direction2 [in] Segunda dirección en grados.
 * @return Valor absoluto de la diferencia de las dos direcciones.
 */
double veins_proj::getDirectionDifference(double direction1,
		double direction2) {
	double directionDifference = std::abs(direction1 - direction2);

	if (directionDifference > 180.0)
		directionDifference = 360.0 - directionDifference;

	return directionDifference;
}

//! Verifica si una ubicación vial se encuentra dentro del dominio de la arista.
/*!
 * La ubicación vial se encuentra dentro del dominio de la arista si
 * la distancia a esta es menor a 20 metros.
 *
 * @param locationOnRoadNetwork [in] Ubicación vial a verificar.
 * @return `true` si la distancia a la arista es menor que 20.
 */
bool veins_proj::inEdgeDomain(
		const LocationOnRoadNetwork &locationOnRoadNetwork) {
	if (locationOnRoadNetwork.distanceToVertexA < 0)
		return false;

	if (locationOnRoadNetwork.distanceToVertexB < 0)
		return false;

	if (locationOnRoadNetwork.distanceToEdge > 20.0)
		return false;

	return true;
}

//! Verifica si una ubicación vial se encuentra dentro del radio de
//! proximidad de un vértice.
/*!
 * La ubicación se encuentra dentro del radio de proximidad del vértice si
 * su distancia a este es menor a 10 metros.
 *
 * @param locationOnRoadNetwork [in] Ubicación vial a verificar.
 * @param vertex [in] Vértice de referencia.
 * @param graph [in] Grafo al que pertenece el vértice.
 * @return `true` si la ubicación se encuentra dentro del radio de proximidad
 * del vértice.
 */
bool veins_proj::inVertex(
		const LocationOnRoadNetwork &locationOnRoadNetwork, Vertex vertex,
		const Graph &graph) {
	const Edge &edge = locationOnRoadNetwork.edge;
	Vertex vertexA = boost::source(edge, graph);
	Vertex vertexB = boost::target(edge, graph);
	const double &distanceToVertexA = locationOnRoadNetwork.distanceToVertexA;
	const double &distanceToVertexB = locationOnRoadNetwork.distanceToVertexB;

	double distanceToVertex;
	if (vertex == vertexA)
		distanceToVertex = distanceToVertexA;

	else if (vertex == vertexB)
		distanceToVertex = distanceToVertexB;

	else
		return false;

	if (distanceToVertex < 10)
		return true;

	return false;
}

//! Calcula el peso de una arista a partir de otra.
/*!
 * El peso de la arista _A_ desde la arista _B_, cuyo vértice en común es _v_,
 * es la diferencia entre la dirección de la arista _A_ desde el vértice _v_
 * y la dirección de la arista _B_ hacia el vértice _v_.
 *
 * Para calcular el peso, ambas aristas deben ser adyacentes.
 *
 * @param edge [in] Arista cuyo peso se va a calcular.
 * @param sourceEdge [in] Arista desde la que se calcula el peso.
 * @param graph [in] Grafo al que pertenecen ambas aristas.
 * @return Peso de la arista
 */
double veins_proj::getEdgeWeight(Edge edge, Edge sourceEdge,
		const Graph &graph) {
	Edge &f = edge;
	Edge &e = sourceEdge;
	Vertex vertexAE = boost::source(e, graph);
	Vertex vertexBE = boost::target(e, graph);
	if (sortedVertices(vertexAE, vertexBE, graph))
		boost::swap(vertexAE, vertexBE);

	Vertex vertexAF = boost::source(f, graph);
	Vertex vertexBF = boost::target(f, graph);
	if (sortedVertices(vertexAF, vertexBF, graph))
		boost::swap(vertexAF, vertexBF);

	double directionE, directionF;

	if (vertexBE == vertexAF) {
		directionE = graph[e].direction1;
		directionF = graph[f].direction1;

	} else if (vertexBE == vertexBF) {
		directionE = graph[e].direction1;
		directionF = graph[f].direction2;

	} else if (vertexAE == vertexAF) {
		directionE = graph[e].direction2;
		directionF = graph[f].direction1;

	} else if (vertexAE == vertexBF) {
		directionE = graph[e].direction2;
		directionF = graph[f].direction2;
	}

	return getDirectionDifference(directionE, directionF);
}

//! Obtiene la distancia en metros de una ubicación vial hacia un vértice.
/*!
 * Para obtener la distancia, el vértice debe ser parte de la arista de la
 * ubicación vial.
 *
 * @param locationOnRoadNetwork [in] Ubicación vial cuya distancia al vértice
 * se va a obtener.
 * @param vertex [in] Vértice al que se va a obtener la distancia.
 * @param graph [in] Grafo al que pertenece el vértice.
 * @return
 */
double veins_proj::getDistanceToVertex(
		const LocationOnRoadNetwork &locationOnRoadNetwork, Vertex vertex,
		const Graph &graph) {
	const Edge &edge = locationOnRoadNetwork.edge;
	Vertex vertexA = boost::source(edge, graph);
	Vertex vertexB = boost::target(edge, graph);
	const double &distanceToVertexA = locationOnRoadNetwork.distanceToVertexA;
	const double &distanceToVertexB = locationOnRoadNetwork.distanceToVertexB;

	if (vertex == vertexA)
		return distanceToVertexA;

	else if (vertex == vertexB)
		return distanceToVertexB;

	return std::numeric_limits<double>::infinity();
}
