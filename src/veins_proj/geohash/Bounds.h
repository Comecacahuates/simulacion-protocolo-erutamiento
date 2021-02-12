/*
 * Region.h
 *
 *  Created on: Jun 3, 2020
 *      Author: adrian
 */

#pragma once

#include "veins_proj/veins_proj.h"
#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/Math.hpp>
#include <omnetpp.h>


namespace veins_proj {


class Bounds: omnetpp::cObject {

protected:
    double north;
    double east;
    double south;
    double west;

public:
    Bounds();
    Bounds(double north, double east, double south, double west);
    virtual ~Bounds();

    double getNorth() const { return this->north; }
    double getEast() const { return this->east; }
    double getSouth() const { return this->south; }
    double getWest() const { return this->west; }
    void setBounds(double north, double east, double south, double west);
    void setNorth(double north);
    void setEast(double east);
    void setSouth(double south);
    void setWest(double west);

public:
    bool contains(const double &lat, const double &lon) const;
    bool contains(const GeographicLib::GeoCoords &location) const { return contains(location.Latitude(), location.Longitude()); }
};


} // namespace veins_proj
