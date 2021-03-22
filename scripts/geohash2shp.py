#!/usr/bin/env python3
"""!
Genera un archivo Shapefile con rectángulos que representan regiones Geohash.

Uso: geohash2shp.py 9g3qx 6 ../roadnetwork-database/geohash

@file: geohash2shp.py
@author: Adrián Juárez Monroy
"""

import sys

from geolib import geohash as gh

import shapefile as shp

base32 = '0123456789bcdefghjkmnpqrstuvwxyz'


def children_geohashes(parent_geohash, children_geohash_length):
    """!
    Genera todos los códigos Geohash hijos de una longitud específica
    a partir de un código Geohash padre.
    
    @param parent_geohash: Código Geohash padre. Es el prefijo de
    los códigos Geohash hijos que se van a generar.
    @param children_geohash_length: Longitud total de los códigos Geohash hijos.
    @return: Lista de códigos Geohash.
    """

    """
    Se verifica que la longitd de los códigos hijos sea mayor
    a la del código padre.
    """
    if children_geohash_length <= len(parent_geohash):
        raise ValueError("La longitud de los códigos Geohash hijos debe ser \
        mayor a la del código Geohash padre.")

    geohashes = [parent_geohash]
    for i in range(children_geohash_length - len(parent_geohash)):
        n_geohashes = len(geohashes)
        geohashes *= len(base32)
        for j in range(len(base32)):
            for k in range(n_geohashes):
                geohashes[j * n_geohashes + k] += base32[j]
    return geohashes


def save_shp(shapefile, geohashes):
    """!
    Guarda los códigos Geohash como polígonos en un archivo Shapefile.
    Cada polígono tiene el campo GEOHASH, que indica el código Geohash,
    y los campos LAT_N, LAT_S, LON_E, LON_W, que indican los límites
    de la región.
    
    @param shapefile: Ruta del archivo Shapefile a escribir.
    @param geohashes: Códigos Geohash.
    """
    shapefile_writer = shp.Writer(shapefile, shapeType=shp.POLYGON)
    shapefile_writer.autoBalance = 1
    shapefile_writer.field('GEOHASH', 'C', size=10)
    shapefile_writer.field('LAT_N', 'N', decimal=10)
    shapefile_writer.field('LAT_S', 'N', decimal=10)
    shapefile_writer.field('LON_E', 'N', decimal=10)
    shapefile_writer.field('LON_W', 'N', decimal=10)

    for geohash in geohashes:
        (lat_s, lon_w), (lat_n, lon_e) = gh.bounds(geohash)
        shapefile_writer.poly([[
            [lon_w, lat_s],
            [lon_e, lat_s],
            [lon_e, lat_n],
            [lon_w, lat_n],
            [lon_w, lat_s], ]])
        shapefile_writer.record(geohash, lat_n, lat_s, lon_e, lon_w)

    shapefile_writer.close()


if __name__ == '__main__':
    if len(sys.argv) != 4:
        raise ValueError("Proporciona el código Geohash padre, la longitud \
        de los códigos Geohash hijos y la ruta donde se va a guardar \
        el archivo Shapefile.")

    parent_geohash = sys.argv[1]
    children_geohash_length = int(sys.argv[2])
    shapefile_dir = sys.argv[3]

    geohashes = children_geohashes(parent_geohash, children_geohash_length)
    save_shp(
        '{}/{}-{}'.format(shapefile_dir, parent_geohash, children_geohash_length),
        geohashes)
