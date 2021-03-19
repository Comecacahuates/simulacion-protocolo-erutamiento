#!/usr/bin/env python3
"""!
Convierte los archivos Shapefile de las redes viales en archivos XML
que se utilizan en la simulación para la base de datos de redes viales.

@file: shp2xml.py
@author: Adrián Juárez Monroy
"""

import math
import os
import sys
from xml.dom import minidom

from numpy import shape
from numpy.core.fromnumeric import _amin_dispatcher

import shapefile as shp


class Vertex:
    """!
    Clase que representa un vértice de la red vial.
    """

    def __init__(self, id, lat, lon, adjacency):
        """!
        Constructor.

        @param id: Identificador.
        @param lat: Latitud de la ubicación.
        @param lon: Longitud de la ubicación.
        @param adjacency: Tipo de adyacencia.
        """
        self.id = id
        self.lat = lat
        self.lon = lon
        self.adjacency = adjacency


class Edge:
    """!
    Clase que representa una arista de la red vial.
    """

    def __init__(self, vertex_a, vertex_b):
        """!
        Constructor.
        
        @param vertex_a: Primer vértice.
        @param vertex_b: Segundo vértice.
        """
        self.vertex_a = vertex_a
        self.vertex_b = vertex_b


def read_vertices(shapefile):
    """!
    Lee el archivo Shapefile que contiene los vértices.
    
    @param shapefile: Ruta del archivo Shapefile a leer.    
    @return: Lista de vértices.
    """
    vertices = []

    shapefile_reader = shp.Reader(shapefile)
    for shape_record in shapefile_reader.shapeRecords():
        if len(shape_record.shape.points) > 0:
            lon, lat = shape_record.shape.points[0]
            id, adjacency = shape_record.record
            vertices.append(Vertex(id, lat, lon, adjacency))

    shapefile_reader.close()
    return sorted(vertices, key=lambda v: v.id)


def read_edges(shapefile, vertices):
    """!
    Lee el archivo Shapefile que contiene las aristas.
    
    @param shapefile: Ruta del archivo Shapefile a leer.
    @param vertices: Lista de vértices.
    @return: Lista de aristas.
    """
    edges = []

    shapefile_reader = shp.Reader(shapefile)
    for shape_record in shapefile_reader.shapeRecords():
        for i in range(len(shape_record.shape.points) - 1):
            (lon_a, lat_a) = shape_record.shape.points[i]
            (lon_b, lat_b) = shape_record.shape.points[i + 1]
            vertex_a = get_closest_vertex(lat_a, lon_a, vertices)
            vertex_b = get_closest_vertex(lat_b, lon_b, vertices)
            edges.append(Edge(vertex_a, vertex_b))

    shapefile_reader.close()
    return edges


def get_closest_vertex(lat, lon, vertices):
    """!
    Obtiene el vértice más cercano a una ubicación.
    
    @param lat: Latitud de la ubicación de interés.
    @param lon: Longitud de la ubicación de interés.
    @param vertices: Lista de vértices en la que se busca.
    @return: Vértice más cercano a la ubicación indicada.
    """
    min_distance = math.hypot(vertices[0].lon - lon, vertices[0].lat - lat)
    closest_vertex = vertices[0]

    for vertex in vertices:
        distance = math.hypot(vertex.lon - lon, vertex.lat - lat)
        if min_distance > distance:
            min_distance = distance
            closest_vertex = vertex

    return closest_vertex


def save_xml(xml_file, vertices, edges):
    """!
    Guarda la red vial en un archivo XML.
    
    @param xml_:file: Ruta del archivo XML a escribir.
    @param vertices: Lista de vértices.
    @param edges: Lista de aristas.
    """
    xml_doc = minidom.Document()

    """ Se crea el nodo raíz de la red vial. """
    roadnetwork_node = xml_doc.createElement('roadnetwork')
    xml_doc.appendChild(roadnetwork_node)

    """ Se crea el nodo de los vértices. """
    vertices_node = xml_doc.createElement('vertices')
    vertices_node.setAttribute('count', str(len(vertices)))
    roadnetwork_node.appendChild(vertices_node)

    """ Se crea un nodo para cada vértice. """
    for vertex in vertices:
        vertex_node = xml_doc.createElement('vertex')
        vertex_node.setAttribute('id', str(vertex.id))
        vertex_node.setAttribute('lat', '{0:.9f}'.format(vertex.lat))
        vertex_node.setAttribute('lon', '{0:.9f}'.format(vertex.lon))
        vertex_node.setAttribute('adjacency', str(vertex.adjacency))
        vertices_node.appendChild(vertex_node)

    """ Se crea el nodo de las aristas. """
    edges_node = xml_doc.createElement('edges')
    edges_node.setAttribute('count', str(len(edges)))
    roadnetwork_node.appendChild(edges_node)

    """ Se crea un nodo para cada arista. """
    for edge in edges:
        edge_node = xml_doc.createElement('edge')
        edge_node.setAttribute('vertex-a', str(edge.vertex_a.id))
        edge_node.setAttribute('vertex-b', str(edge.vertex_b.id))
        edges_node.appendChild(edge_node)

    with open(xml_file, 'w') as file:
        file.write(xml_doc.toprettyxml(indent="    "))


if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise ValueError("Proporciona la ruta del directorio donde \
        se encuentran los archivos Shapefile y del directorio donde se \
        van a guardar los archivos XML.")

    shapefile_dir = sys.argv[1]
    xml_dir = sys.argv[2]

    for geohash in os.listdir(shapefile_dir):
        vertices_shapefile = '{}/{}/vertices'.format(shapefile_dir, geohash)
        edges_shapefile = '{}/{}/edges'.format(shapefile_dir, geohash)
        xml_file = '{}/{}.xml'.format(xml_dir, geohash)
        vertices = read_vertices(vertices_shapefile)
        edges = read_edges(edges_shapefile, vertices)
        save_xml(xml_file, vertices, edges)
        print('{} guardado.'.format(xml_file))
