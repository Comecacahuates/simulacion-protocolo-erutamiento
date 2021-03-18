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
 * @file Bounds.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include "veins_proj/veins_proj.h"
#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/Math.hpp>
#include <omnetpp.h>

namespace veins_proj {

/*!
 * @brief Clase que representa los límites de una región rectangular.
 */
class Bounds: omnetpp::cObject {

private:

    /*
     * Atributos
     */
    //! Latitud norte.
    double north;
    //! Longitud este
    double east;
    //! Latitud sur.
    double south;
    //! Longitud oeste.
    double west;

public:

    /*
     * Constructores.
     */
    /*!
     * @brief Crear una región que abarca el rango completo
     * de latitud y longitud.
     */
    Bounds();
    /*!
     * @brief Crea una región con los límites indicados.
     *
     * @param north [in] Latitud norte.
     * @param east  [in] Longitud este.
     * @param south [in] Latitud sur.
     * @param west  [in] Longitud oeste.
     */
    Bounds(double north, double east, double south, double west);

    /*
     * Acceso a los atributos.
     */
    /*!
     * @brief Acceso a la latitud norte.
     *
     * @return Latitud norte.
     */
    double getNorth() const {
        return this->north;
    }
    /*!
     * @brief Acceso a la longitud este.
     *
     * @return Longitud este.
     */
    double getEast() const {
        return this->east;
    }
    /*!
     * @brief Acceso a la latitud sur.
     *
     * @return Latitud sur.
     */
    double getSouth() const {
        return this->south;
    }
    /*!
     * @brief Acceso a la longitud oeste.
     *
     * @return Longitud oeste.
     */
    double getWest() const {
        return this->west;
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
    void setBounds(double north, double east, double south, double west);
    /*!
     * @brief Modificar la latitud norte.
     *
     * @param north [in] Latitud norte.
     */
    void setNorth(double north);
    /*!
     * @brief Modificar la longitud este.
     *
     * @param east [in] Longitud este.
     */
    void setEast(double east);
    /*!
     * @brief Modificar la latitud sur.
     *
     * @param south [in] Latitud sur.
     */
    void setSouth(double south);
    /*!
     * @brief Modificar la longitud oeste.
     *
     * @param west [in] Longitud oeste.
     */
    void setWest(double west);

    /*!
     * @brief Verificar si una ubicación geográfica está adentro de la región.
     *
     * @param lat [in] Latitud.
     * @param lon [in] Longitud.
     * @return `true` si la ubicación está adentro de la región.
     */
    bool contains(const double &lat, const double &lon) const;
    /*!
     * @brief Verificar si una ubicación geográfica está adentro de la región.
     *
     * @param location [in] Ubicación geográfica.
     * @return `true` si la ubicación está adentro de la región.
     */
    bool contains(const GeographicLib::GeoCoords &location) const {
        return contains(location.Latitude(), location.Longitude());
    }
};

}    // namespace veins_proj
