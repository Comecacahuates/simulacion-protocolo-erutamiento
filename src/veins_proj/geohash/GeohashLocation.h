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
 * @file GeohashLocation.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include <GeographicLib/Math.hpp>
#include <GeographicLib/GeoCoords.hpp>
#include "veins_proj/veins_proj.h"
#include <omnetpp.h>
#include "veins_proj/geohash/Bounds.h"
#include <string>

namespace veins_proj {

/*!
 * @brief Clase que representa ubicaciones y regiones en codificación Geohash.
 */
class GeohashLocation: omnetpp::cObject {

private:

    /*
     * Datos auxiliares.
     */
    //! Conjunto de símbolos base 32 con los que se forman los códigos Geohash.
    static const std::string base32;
    //! Símbolos que se encuentran en los bordes de las regiones adyacentes.
    static const std::string neighbour[4][2];
    //! Símbolos que se encuentran en los bordes de las regiones.
    static const std::string border[4][2];

public:

    /*!
     * Tipos de adyacencia entre regiones Geohash.
     */
    enum Adjacency {
        //! Ninguna adyacencia.
        NONE = -1,
        //! Adyacencia al norte.
        NORTH = 0,
        //! Adyacencia al este.
        EAST = 1,
        //! Adyacencia al sur.
        SOUTH = 2,
        //! Adyacencia al oeste.
        WEST = 3,
    };

private:

    /*
     * Atributos.
     */
    //! Ubicación geográfica.
    GeographicLib::GeoCoords location;
    //! Código Geohash.
    std::string geohash;    // TODO Cambiar por arreglo de caracteres.
    //! Longitus del código Geohash.
    size_t geohashLength;
    //! Cadena de bits del código Geohash.
    uint64_t bits;
    //! Límites de la región.
    Bounds bounds;

public:

    /*
     * Contructores.
     */
    /*!
     * @brief Contruir ubicación Geohash nula.
     */
    GeohashLocation();
    /*!
     * @brief Contruir ubicación Geohash a partir de una ubicación geográfica
     * y la longitud del código.
     *
     * @param location      [in] Ubicación geográfica.
     * @param geohashLength [in] Longitud del código Geohash.
     */
    GeohashLocation(const GeographicLib::GeoCoords &location,
            const size_t geohashLength);
    /*!
     * @brief Contruir ubicación Geohash a partir de un código Geohash.
     *
     * @param geohash [in] Código Geohash.
     */
    GeohashLocation(const std::string &geohash);
    /*!
     * @brief Construir ubicación Geohash a partir de una cadena de bits
     * y la longitud del código.
     *
     * @param bits          [in] Cadena de bits.
     * @param geohashLength [in] Longitud del código Geohash.
     */
    GeohashLocation(uint64_t bits, const size_t geohashLength);

    /*
     * Sobrecarga de operadores.
     */
    /*!
     * @brief Comparar dos ubicaciones Geohash para saber si son iguales.
     *
     * @param other [in] Ubicación Geohash con la que se compara.
     * @return `true` si son iguales.
     */
    bool operator ==(const GeohashLocation &other) {
        return geohash == other.geohash;
    }
    /*!
     * @brief Comparar dos ubicaciones Geohash para saber si son diferentes.
     *
     * @param other [in] Ubicación Geohash con la que se compara.
     * @return `true` si son diferentes.
     */
    bool operator !=(const GeohashLocation &other) {
        return geohash != other.geohash;
    }

    /*
     * Acceso a los atributos.
     */
    /*!
     * @brief Obtener la ubicación geográfica.
     *
     * @return Ubicación geográfica.
     */
    const GeographicLib::GeoCoords& getLocation() const {
        return location;
    }
    /*!
     * @brief Obtener el código Geohash.
     *
     * @return Cófigo Geohash.
     */
    const std::string& getGeohash() const {
        return geohash;
    }
    /*!
     * @brief Obtener la longitud del código Geohash.
     *
     * @return Longitud del código Geohash.
     */
    size_t getGeohashLength() const {
        return geohash.length();
    }
    /*!
     * @brief Obtener cadena de bits del código Geohash.
     * @return
     */
    uint64_t getBits() const {
        return bits;
    }
    /*!
     * @brief Obtener los límites de la región Geohash.
     *
     * @return Límites de la región Geohash.
     */
    const Bounds& getBounds() const {
        return bounds;
    }

    /*
     * Modificación de atributos.
     */
    /*!
     * @brief Modificar la ubicación Geográfica.
     *
     * @param location [in] Nueva ubicación Geográfica.
     */
    void setLocation(const GeographicLib::GeoCoords &location);
    /*!
     * @brief Modificar el código Geohash.
     *
     * @param geohash [in] Nuevo código Geohash.
     */
    void setGeohash(const std::string &geohash);
    /*!
     * @brief Modificar la cadena de bits del código Geohash.
     *
     * @param bits [in] Nueva cadena de bits.
     */
    void setBits(uint64_t bits);

    /*
     * Ubicación Geohash nula.
     */
    /*!
     * @brief Verificar si la ubicación Geohash es nula.
     *
     * @return `true` si la ubiación Geohash es nula.
     */
    bool isNull() const {
        return geohash.length() == 0;
    }
    /*!
     * @brief Establecer la ubicación Geohash como nula.
     */
    void setNull();

    /*
     * Operaciones geográficas.
     */
    /*!
     * @brief Calcular distancia a ubicación.
     *
     * @param location [in] Ubicación cuya distancia se busca.
     * @return Distancia en metros.
     */
    double getDistance(const GeographicLib::GeoCoords &location) const;
    /*!
     * @brief Calcular distancia a otra ubicación Geohash.
     *
     * @param geohashLocation [in] Ubicación Geohash cuya distancia se busca.
     * @return Distancia en metros.
     */
    double getDistance(const GeohashLocation &geohashLocation) const {
        return getDistance(geohashLocation.getLocation());
    }
    /*!
     * @brief Verificar si una ubicación se encuentra dentro de la región.
     *
     * @param location [in] Ubicación que se verifica.
     * @return `true` si la ubicación se encuentra dentro de la región.
     */
    bool contains(const GeographicLib::GeoCoords &location) const {
        return bounds.contains(location);
    }
    /*!
     * @brief Verificar si una ubicación Geohash se encuentra dentro de la
     * región.
     *
     * @param geohashLocation [in] Ubicación Geohash que se verifica.
     * @return `true` si la ubicación Geohash se encuentra dentro de la región.
     */
    bool contains(const GeohashLocation &geohashLocation) const {
        return bounds.contains(geohashLocation.location);
    }
    /*!
     * @brief Obtener una región Geohash adyacente.
     *
     * @param adjacency [in] Dirección de adyacencia.
     * @return Región Geohash adyacente.
     *  TODO Mover al archivo GeohashLocation.cc
     */
    GeohashLocation getAdjacentGeohashRegion(const Adjacency &adjacency) const;

    /*
     * Métodos estáticos.
     */
    /*!
     * @brief Codificar una ubicación geográfica.
     *
     * @param location      [in]  Ubicación geográfica a codificar.
     * @param geohashLength [in]  Longitud del código Geohash.
     * @param geohash       [out] Código Geohash.
     * @param center        [out] Centro de la región Geohash.
     * @param bounds        [out] Límites de la región Geohash.
     * @param bits          [out] Cadena de bits del código Geohash.
     */
    static void encode(const GeographicLib::GeoCoords &location,
            size_t geohashLength, std::string &geohash,
            GeographicLib::GeoCoords &center, Bounds &bounds, uint64_t &bits);
    /*!
     * @brief Deodificar cadena de bits.
     *
     * @param bits          [in]  Cadena de bits a decodificar.
     * @param geohashLength [in]  Longitud del código Geohash.
     * @param geohash       [out] Código Geohash.
     * @param center        [out] Centro  de la región Geohash.
     * @param bounds        [out] Límites de la región Geohash.
     */
    static void decode(const uint64_t &bits, size_t geohashLength,
            std::string &geohash, GeographicLib::GeoCoords &center,
            Bounds &bounds);
    /*!
     * @brief Decodificar código Geohash.
     *
     * @param geohash [in]  Código Geohash a decodificar.
     * @param center  [out] Centro de la región Geohash.
     * @param bounds  [out] Límites de la región Geohash.
     * @param bits    [out] Cadena de bits del código Geohash.
     */
    static void decode(const std::string &geohash,
            GeographicLib::GeoCoords &center, Bounds &bounds, uint64_t &bits);
    /*!
     * @brief Obtener una región Geohash adyacente.
     *
     * @param geohash         [in] Código Geohash del que se busca
     * la región adyacente
     * @param adjacency       [in] Dirección de la adyacencia.
     * @param adjacentGeohash [out] Código Geohash de la región adyacente.
     */
    static void adjacentGeohashRegion(const std::string &geohash,
            const Adjacency adjacency, std::string &adjacentGeohash);
};

}    // namespace veins_proj
