/*
 * Region.cc
 *
 *  Created on: Jun 3, 2020
 *      Author: adrian
 */

#include <GeographicLib/Constants.hpp>
#include "veins_proj/geohash/Bounds.h"

using namespace veins_proj;


Bounds::Bounds() {
    north =   90.0;
    east =   180.0;
    south =  -90.0;
    west =  -180.0;
}


Bounds::Bounds(double north, double east, double south, double west) {
    this->setBounds(north, east, south, west);
}


Bounds::~Bounds() {}


void Bounds::setBounds(double north, double east, double south, double west) {
    this->north =   90.0;
    this->east =   180.0;
    this->south =  -90.0;
    this->west =  -180.0;

    setNorth(north);
    setEast(east);
    setSouth(south);
    setWest(west);
}


void Bounds::setNorth(double north) {
    if (north < -90.0 || 90.0 < north)
        throw GeographicLib::GeographicErr("North latitude out of range (-90, 90)");

    if (north < this->south)
        throw GeographicLib::GeographicErr("North less than south");

    this->north = north;
}


void Bounds::setEast(double east) {
    if (east < -180.0 || 180.0 < east)
        throw GeographicLib::GeographicErr("East longitude out of range (-180, 180)");

    if (east < this->west)
        throw GeographicLib::GeographicErr("East less than west");

    this->east = east;
}


void Bounds::setSouth(double south) {
    if (south < -90.0 || 90.0 < south)
        throw GeographicLib::GeographicErr("South latitude out of range (-90, 90)");

    if (south > this->north)
        throw GeographicLib::GeographicErr("South greater than north");

    this->south = south;
}


void Bounds::setWest(double west) {
    if (west < -180.0 || 180.0 < west)
        throw GeographicLib::GeographicErr("West longitude out of range (-180, 180)");

    if (west > this->east)
        throw GeographicLib::GeographicErr("West greater than east");

    this->west = west;
}


bool Bounds::contains(const double &lat, const double &lon) const {
    return south <= lat && lat <= north && west <= lon && lon <= east;
}
