/*
 * RoadNetworkGraph.h
 *
 *  Created on: Dec 31, 2020
 *      Author: adrian
 */

#pragma once

#include <GeographicLib/GeoCoords.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/tuple/tuple.hpp>
#include "veins_proj/geohash/GeohashLocation.h"
#include <omnetpp.h>
#include <vector>
#include <set>

namespace veins_proj {

//! Datos de cada vértice.
struct VertexData {
	//! Ubicación del vértice.
	GeographicLib::GeoCoords location;
	//! Tipo de _gateway_.
	GeohashLocation::Direction gatewayType;
};

//! Datos de cada arista.
struct EdgeData {
	//! Peso de la arista.
	double weight;
	//! Longitud de la arista en metros.
	double length;
	//! Dirección del segundo vértice respecto al primero.
	double direction1;
	//! Dirección del primer vértice respecto a segundo.
	double direction2;
};

//! Grafo vial.
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
		VertexData, EdgeData> Graph;
//! Vértice del grafo vial.
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
//! Iterador de vértices para grafo vial.
typedef boost::graph_traits<Graph>::vertex_iterator VertexIterator;
//! Arista del grafo vial.
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
//! Iterador de aristas para grafo vial.
typedef boost::graph_traits<Graph>::edge_iterator EdgeIterator;
//! Iterador de aristas de salida para grafo vial.
typedef boost::graph_traits<Graph>::out_edge_iterator OutEdgeIterator;
//! Iterador de aristas de entrada para grafo vial.
typedef boost::graph_traits<Graph>::in_edge_iterator InEdgeIterator;

//! Vector de vértices.
typedef std::vector<Vertex> VertexVector;
//! Iterador de vértices para vector.
typedef VertexVector::iterator VertexVectorIterator;
//! Iterador de vértices para vector constante.
typedef VertexVector::const_iterator VertexVectorConstIterator;

//! Conjunto de vértices.
typedef std::set<Vertex> VertexSet;
//! Iterador de vértices para conjunto.
typedef VertexSet::iterator VertexSetIterator;
//! Iterador de vértices para conjunto constante.
typedef VertexSet::const_iterator VertexSetConstIterator;

//! Vector de aristas.
typedef std::vector<Edge> EdgeVector;
//! Iterador de aristas para vector.
typedef EdgeVector::iterator EdgeVectorIterator;
//! Iterador de aristas para vector constante.
typedef EdgeVector::const_iterator EdgeVectorConstIterator;

//! Conjunto de aristas.
typedef std::set<Edge> EdgeSet;
//! Iterador de aristas para conjunto.
typedef EdgeSet::iterator EdgeSetIterator;
//! Iterador de aristas para conjunto constante.
typedef EdgeSet::const_iterator EdgeSetConstIterator;

//! Ubicación vial de un objeto.
struct LocationOnRoadNetwork {
	//! Arista en la que se ubica el objeto.
	Edge edge;
	//! Distancia hacia la arista.
	double distanceToEdge;
	//! Sistancia hacia el primer vértice de la arista.
	double distanceToVertex1;
	//! Distancia hacia el segundo vértice de la arista.
	double distanceToVertex2;
};

//! Determina si la ubicación del primer vértice es menor a la del segundo.
/*!
 * Para que la ubicación del primer vértice sea menor, su latitud debe ser
 * menor a la del segundo. Si las latitudes son iguales, la longitud debe ser
 * menor.
 *
 * \param vertexA [in] Primer vértice cuya ubicación se va a comparar.
 * \param vertexB [in] Segundo vértice cuya ubicación se va a comparar.
 * \param graph [in] Grafo al que pertenecen los dos vértices.
 * \return `true` si la ubicación del primer vértice es menor a la del
 * segundo.
 */
bool sortedVertices(Vertex vertexA, Vertex vertexB, const Graph &graph);

//! Determina si un vértice es _gateway_.
/*!
 * \param vertex [in] Vértice que se va a revisar.
 * \param graph [in] Grafo al que pertenece el vértice.
 * \return `true` en caso de que el vértice sea _gateway_.
 */
bool isGateway(Vertex vertex, const Graph &graph);

//! Verifica si dos direcciones acimutales coinciden.
/*!
 * Para que dos direcciones coincidan, su diferencia debe ser menor
 * a 15 grados.
 *
 * \param direction1 [in] Primera dirección en grados.
 * \param direction2 [in] Segunda dirección en grados.
 * \return `true` si las dos direcciones coinciden.
 */
bool directionMatches(double direction1, double direction2);

//! Calcula la diferencia de dos direcciones acimutales.
/*!
 * La diferencia es el ángulo menor, en grados, que forman dos direcciones.
 *
 * \param direction1 [in] Primera dirección en grados.
 * \param direction2 [in] Segunda dirección en grados.
 * \return Valor absoluto de la diferencia de las dos direcciones.
 */
double getDirectionDifference(double direction1, double direction2);

//! Verifica si una ubicación vial se encuentra dentro del dominio de la arista.
/*!
 * La ubicación vial se encuentra dentro del dominio de la arista si
 * la distancia a esta es menor a 20 metros.
 *
 * \param locationOnRoadNetwork [in] Ubicación vial a verificar.
 * \return `true` si la distancia a la arista es menor que 20.
 */
bool inEdgeDomain(const LocationOnRoadNetwork &locationOnRoadNetwork);

//! Verifica si una ubicación vial se encuentra dentro del radio de
//! proximidad de un vértice.
/*!
 * La ubicación se encuentra dentro del radio de proximidad del vértice si
 * su distancia a este es menor a 10 metros.
 *
 * \param locationOnRoadNetwork [in] Ubicación vial a verificar.
 * \param vertex [in] Vértice de referencia.
 * \param graph [in] Grafo al que pertenece el vértice.
 * \return `true` si la ubicación se encuentra dentro del radio de proximidad
 * del vértice.
 */
bool inVertexProximityRadius(const LocationOnRoadNetwork &locationOnRoadNetwork,
		Vertex vertex, const Graph &graph);

//! Calcula el peso de una arista a partir de otra.
/*!
 * El peso de la arista _A_ desde la arista _B_, cuyo vértice en común es _v_,
 * es la diferencia entre la dirección de la arista _A_ desde el vértice _v_
 * y la dirección de la arista _B_ hacia el vértice _v_.
 *
 * Para calcular el peso, ambas aristas deben ser adyacentes.
 *
 * \param edge [in] Arista cuyo peso se va a calcular.
 * \param sourceEdge [in] Arista desde la que se calcula el peso.
 * \param graph [in] Grafo al que pertenecen ambas aristas.
 * \return Peso de la arista
 */
double getEdgeWeight(Edge edge, Edge sourceEdge, const Graph &graph);

//! Obtiene la distancia en metros de una ubicación vial hacia un vértice.
/*!
 * Para obtener la distancia, el vértice debe ser parte de la arista de la
 * ubicación vial.
 *
 * \param locationOnRoadNetwork [in] Ubicación vial cuya distancia al vértice
 * se va a obtener.
 * \param vertex [in] Vértice al que se va a obtener la distancia.
 * \param graph [in] Grafo al que pertenece el vértice.
 * \return
 */
double getDistanceToVertex(const LocationOnRoadNetwork &locationOnRoadNetwork,
		Vertex vertex, const Graph &graph);

} // namespace veins_proj
