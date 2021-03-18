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
 * @file Bounds.cc
 * @author Adrián Juárez Monroy
 */

#include <GeographicLib/Constants.hpp>
#include "veins_proj/geohash/Bounds.h"

using namespace veins_proj;

/*
 * Constructores.
 */

/*!
 * @brief Crear una región que abarca el rango completo
 * de latitud y longitud.
 */
Bounds::Bounds() {
    north = 90.0;
    east = 180.0;
    south = -90.0;
    west = -180.0;
}

/*!
 * @brief Crea una región con los límites indicados.
 *
 * @param north [in] Latitud norte.
 * @param east  [in] Longitud este.
 * @param south [in] Latitud sur.
 * @param west  [in] Longitud oeste.
 */
Bounds::Bounds(double north, double east, double south, double west) {
    this->setBounds(north, east, south, west);
}

/*
 * Modificación de los atributos.
 */

/*!
 * @brief Modificar los límites de la región.
 *
 * @param north [in] Latitud norte.
 * @param east  [in] Longitud este.
 * @param south [in] Latitud sur.
 * @param west  [in] Longitud oeste.
 */
void Bounds::setBounds(double north, double east, double south, double west) {
    this->north = 90.0;
    this->east = 180.0;
    this->south = -90.0;
    this->west = -180.0;
    setNorth(north);
    setEast(east);
    setSouth(south);
    setWest(west);
}

/*!
 * @brief Modificar la latitud norte.
 *
 * @param north [in] Latitud norte.
 */
void Bounds::setNorth(double north) {
    if (north < -90.0 || 90.0 < north)
        throw GeographicLib::GeographicErr(
                "North latitude out of range (-90, 90)");

    if (north < this->south)
        throw GeographicLib::GeographicErr("North less than south");

    this->north = north;
}

/*!
 * @brief Modificar la longitud este.
 *
 * @param east [in] Longitud este.
 */
void Bounds::setEast(double east) {
    if (east < -180.0 || 180.0 < east)
        throw GeographicLib::GeographicErr(
                "East longitude out of range (-180, 180)");

    if (east < this->west)
        throw GeographicLib::GeographicErr("East less than west");

    this->east = east;
}

/*!
 * @brief Modificar la latitud sur.
 *
 * @param south [in] Latitud sur.
 */
void Bounds::setSouth(double south) {
    if (south < -90.0 || 90.0 < south)
        throw GeographicLib::GeographicErr(
                "South latitude out of range (-90, 90)");

    if (south > this->north)
        throw GeographicLib::GeographicErr("South greater than north");

    this->south = south;
}

/*!
 * @brief Modificar la longitud oeste.
 *
 * @param west [in] Longitud oeste.
 */
void Bounds::setWest(double west) {
    if (west < -180.0 || 180.0 < west)
        throw GeographicLib::GeographicErr(
                "West longitude out of range (-180, 180)");

    if (west > this->east)
        throw GeographicLib::GeographicErr("West greater than east");

    this->west = west;
}

/*!
 * @brief Verificar si una ubicación geográfica está adentro de la región.
 *
 * @param lat [in] Latitud.
 * @param lon [in] Longitud.
 * @return `true` si la ubicación está adentro de la región.
 */
bool Bounds::contains(const double &lat, const double &lon) const {
    return south <= lat && lat <= north && west <= lon && lon <= east;
}
