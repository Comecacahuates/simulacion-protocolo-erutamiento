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
 * @file GeohashLocation.cc
 * @author Adrián Juárez Monroy
 */

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/Geodesic.hpp>
#include "veins_proj/geohash/GeohashLocation.h"

using namespace veins_proj;

//! Conjunto de símbolos base 32 con los que se forman los códigos Geohash.
const std::string GeohashLocation::base32 = "0123456789bcdefghjkmnpqrstuvwxyz";

//! Símbolos que se encuentran en los bordes de las regiones adyacentes.
const std::string GeohashLocation::neighbour[4][2] =
        { { "p0r21436x8zb9dcf5h7kjnmqesgutwvy",
                "bc01fg45238967deuvhjyznpkmstqrwx" }, {
                "bc01fg45238967deuvhjyznpkmstqrwx",
                "p0r21436x8zb9dcf5h7kjnmqesgutwvy" }, {
                "14365h7k9dcfesgujnmqp0r2twvyx8zb",
                "238967debc01fg45kmstqrwxuvhjyznp" }, {
                "238967debc01fg45kmstqrwxuvhjyznp",
                "14365h7k9dcfesgujnmqp0r2twvyx8zb" } };

//! Símbolos que se encuentran en los bordes de las regiones.
const std::string GeohashLocation::border[4][2] = { { "prxz", "bcfguvyz" }, {
        "bcfguvyz", "prxz" }, { "028b", "0145hjnp" }, { "0145hjnp", "028b" } };

/*
 * Contructores.
 */

/*!
 * @brief Contruir ubicación Geohash nula.
 */
GeohashLocation::GeohashLocation() :
        location(), bounds() {
    geohash = "";
    bits = 0;
}

/*!
 * @brief Contruir ubicación Geohash a partir de una ubicación geográfica
 * y la longitud del código.
 *
 * @param location      [in] Ubicación geográfica.
 * @param geohashLength [in] Longitud del código Geohash.
 */
GeohashLocation::GeohashLocation(const GeographicLib::GeoCoords &location,
        const size_t geohashLength) {
    GeohashLocation::encode(location, geohashLength, geohash, this->location,
            bounds, bits);
    this->location = location;
}

/*!
 * @brief Contruir ubicación Geohash a partir de un código Geohash.
 *
 * @param geohash [in] Código Geohash.
 */
GeohashLocation::GeohashLocation(const std::string &geohash) {
    setGeohash(geohash);
}

/*!
 * @brief Construir ubicación Geohash a partir de una cadena de bits
 * y la longitud del código.
 *
 * @param bits          [in] Cadena de bits.
 * @param geohashLength [in] Longitud del código Geohash.
 */
GeohashLocation::GeohashLocation(const uint64_t bits, const size_t geohashLength) {
    GeohashLocation::decode(bits, geohashLength, geohash, location, bounds);
    this->bits = bits;
}

/*
 * Modificación de atributos.
 */

/*!
 * @brief Modificar la ubicación Geográfica.
 *
 * @param location [in] Nueva ubicación Geográfica.
 */
void GeohashLocation::setLocation(const GeographicLib::GeoCoords &location) {
    GeohashLocation::encode(location, geohash.length(), geohash, this->location,
            bounds, bits);
    this->location = location;
}

/*!
 * @brief Modificar el código Geohash.
 *
 * @param geohash [in] Nuevo código Geohash.
 */
void GeohashLocation::setGeohash(const std::string &geohash) {
    this->geohash = geohash;
    GeohashLocation::decode(geohash, location, bounds, bits);
}

/*!
 * @brief Modificar la cadena de bits del código Geohash.
 *
 * @param bits [in] Nueva cadena de bits.
 */
void GeohashLocation::setBits(uint64_t bits) {
    this->bits = bits;
    GeohashLocation::decode(bits, geohash.length(), geohash, location, bounds);
}

/*
 * Ubicación Geohash nula.
 */

/*!
 * @brief Establecer la ubicación Geohash como nula.
 */
void GeohashLocation::setNull() {
    geohash.clear();
    bits = 0;
    location.Reset(GeographicLib::Math::NaN(), GeographicLib::Math::NaN());
    bounds.setBounds(90, 180, -90, -180);
}

/*
 * Operaciones geográficas.
 */

/*!
 * @brief Calcular distancia a ubicación.
 *
 * @param location [in] Ubicación cuya distancia se busca.
 * @return Distancia en metros.
 */
double GeohashLocation::getDistance(
        const GeographicLib::GeoCoords &location) const {
    const GeographicLib::Geodesic &geod = GeographicLib::Geodesic::WGS84();

    double distance;

    geod.Inverse(this->location.Latitude(), this->location.Longitude(),
            location.Latitude(), location.Longitude(), distance);

    return distance;
}

/*!
 * @brief Obtener una región Geohash adyacente.
 *
 * @param adjacency [in] Dirección de adyacencia.
 * @return Región Geohash adyacente.
 *  TODO Mover al archivo GeohashLocation.cc
 */
GeohashLocation GeohashLocation::getAdjacentGeohashRegion(
        const Adjacency &adjacency) const {
    GeohashLocation geohashLocation;
    std::string adjacentGeohash;
    GeohashLocation::adjacentGeohashRegion(geohash, adjacency, adjacentGeohash);
    return GeohashLocation(adjacentGeohash);
}

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
void GeohashLocation::encode(const GeographicLib::GeoCoords &location,
        size_t geohashLength, std::string &geohash,
        GeographicLib::GeoCoords &center, Bounds &bounds, uint64_t &bits) {
    geohashLength =
            geohashLength < 1 || geohashLength > 12 ? 12 : geohashLength;

    geohash.clear();

    bool evenBit = true;
    unsigned int idx = 0;
    unsigned int bit = 0;

    double lonMid, lonMin = -180, lonMax = 180, latMid, latMin = -90, latMax =
            90;

    bits = 0;
    uint64_t mask = 1;
    mask <<= 63;

    while (geohash.length() < geohashLength) {
        if (evenBit) {
            lonMid = (lonMin + lonMax) / 2.0;
            if (location.Longitude() >= lonMid) {
                idx = idx * 2 + 1;
                lonMin = lonMid;
                bits |= mask;

            } else {
                idx = idx * 2;
                lonMax = lonMid;
            }

        } else {
            latMid = (latMin + latMax) / 2.0;
            if (location.Latitude() >= latMid) {
                idx = idx * 2 + 1;
                latMin = latMid;
                bits |= mask;

            } else {
                idx = idx * 2;
                latMax = latMid;
            }
        }

        evenBit = !evenBit;
        mask >>= 1;

        if (++bit == 5) {
            geohash.push_back(GeohashLocation::base32.at(idx));
            bit = 0;
            idx = 0;
        }
    }

    center = GeographicLib::GeoCoords((latMin + latMax) / 2.0,
            (lonMin + lonMax) / 2.0);
    bounds = Bounds(latMax, lonMax, latMin, lonMin);
}

/*!
 * @brief Deodificar cadena de bits.
 *
 * @param bits          [in]  Cadena de bits a decodificar.
 * @param geohashLength [in]  Longitud del código Geohash.
 * @param geohash       [out] Código Geohash.
 * @param center        [out] Centro  de la región Geohash.
 * @param bounds        [out] Límites de la región Geohash.
 */
void GeohashLocation::decode(const uint64_t &bits, size_t geohashLength,
        std::string &geohash, GeographicLib::GeoCoords &center,
        Bounds &bounds) {
    geohashLength =
            geohashLength < 1 || geohashLength > 12 ? 12 : geohashLength;

    geohash.clear();

    bool evenBit = true;
    unsigned int idx = 0;
    uint64_t bit = 0;

    double lonMid, lonMin = -180, lonMax = 180, latMid, latMin = -90, latMax =
            90;

    uint64_t mask = 1;
    mask <<= 63;

    int nBits = geohashLength * 5;

    for (int i = 0; i < nBits; i++) {
        bit = (bits & mask) >> (63 - i);

        if (evenBit) {
            lonMid = (lonMin + lonMax) / 2.0;
            if (bit == 1) {
                idx = idx * 2 + 1;
                lonMin = lonMid;

            } else {
                idx = idx * 2;
                lonMax = lonMid;
            }

        } else {
            latMid = (latMin + latMax) / 2.0;
            if (bit == 1) {
                idx = idx * 2 + 1;
                latMin = latMid;

            } else {
                idx = idx * 2;
                latMax = latMid;
            }
        }

        evenBit = !evenBit;
        mask >>= 1;

        if ((i + 1) % 5 == 0) {
            geohash.push_back(GeohashLocation::base32.at(idx));
            bit = 0;
            idx = 0;
        }
    }

    center = GeographicLib::GeoCoords((latMin + latMax) / 2.0,
            (lonMin + lonMax) / 2.0);
    bounds = Bounds(latMax, lonMax, latMin, lonMin);
}

/*!
 * @brief Decodificar código Geohash.
 *
 * @param geohash [in]  Código Geohash a decodificar.
 * @param center  [out] Centro de la región Geohash.
 * @param bounds  [out] Límites de la región Geohash.
 * @param bits    [out] Cadena de bits del código Geohash.
 */
void GeohashLocation::decode(const std::string &geohash,
        GeographicLib::GeoCoords &center, Bounds &bounds, uint64_t &bits) {
    bool evenBit = true;
    unsigned int idx = 0;
    uint8_t bit = 0;

    double lonMid, lonMin = -180, lonMax = 180, latMid, latMin = -90, latMax =
            90;

    bits = 0;
    uint64_t mask = 1;
    mask <<= 63;

    for (char c : geohash) {
        idx = GeohashLocation::base32.find(c);
        if (idx == std::string::npos)
            throw GeographicLib::GeographicErr("Invalid geohash");

        for (int i = 4; i >= 0; i--) {
            bit = idx >> i & 1;

            if (evenBit) {
                lonMid = (lonMin + lonMax) / 2.0;
                if (bit == 1) {
                    lonMin = lonMid;
                    bits |= mask;

                } else
                    lonMax = lonMid;

            } else {
                latMid = (latMin + latMax) / 2.0;
                if (bit == 1) {
                    latMin = latMid;
                    bits |= mask;

                } else
                    latMax = latMid;
            }

            evenBit = !evenBit;
            mask >>= 1;
        }
    }

    center = GeographicLib::GeoCoords((latMin + latMax) / 2.0,
            (lonMin + lonMax) / 2.0);
    bounds = Bounds(latMax, lonMax, latMin, lonMin);
}

/*!
 * @brief Obtener una región Geohash adyacente.
 *
 * @param geohash         [in] Código Geohash del que se busca
 * la región adyacente
 * @param adjacency       [in] Dirección de la adyacencia.
 * @param adjacentGeohash [out] Código Geohash de la región adyacente.
 */
void GeohashLocation::adjacentGeohashRegion(const std::string &geohash,
        const Adjacency adjacency, std::string &adjacentGeohash) {
    char lastChar = geohash.back();
    std::string parent = geohash.substr(0, geohash.length() - 1);

    unsigned int type = geohash.length() % 2;

    if (GeohashLocation::border[adjacency][type].find(lastChar)
            != std::string::npos && parent.empty())
        GeohashLocation::adjacentGeohashRegion(parent, adjacency,
                adjacentGeohash);

    parent.push_back(
            GeohashLocation::base32.at(
                    GeohashLocation::neighbour[adjacency][type].find(
                            lastChar)));
    adjacentGeohash = parent;
}
