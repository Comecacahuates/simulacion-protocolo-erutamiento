/*
 * GeohashLocation.h
 *
 *  Created on: Jun 9, 2020
 *      Author: adrian
 */

#pragma once

#include <GeographicLib/Math.hpp>
#include <GeographicLib/GeoCoords.hpp>
#include "veins_proj/veins_proj.h"
#include <omnetpp.h>
#include "veins_proj/geohash/Bounds.h"
#include <string>

namespace veins_proj {


class GeohashLocation: omnetpp::cObject {

protected:
    static const std::string base32;
    static const std::string neighbour[4][2];
    static const std::string border[4][2];

public:
   enum Direction { NONE = -1, NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

protected:
    std::string geohash;
    uint64_t bits;
    GeographicLib::GeoCoords location;
    Bounds bounds;

public:
    GeohashLocation();
    GeohashLocation(const std::string &geohash);
    GeohashLocation(const GeographicLib::GeoCoords &location, unsigned int precision);
    GeohashLocation(uint64_t bits, unsigned int precision);
    ~GeohashLocation();

    bool operator ==(const GeohashLocation &other) { return bits == other.bits && geohash.length() == other.geohash.length(); }
    bool operator !=(const GeohashLocation &other) { return !(*this == other); }

    bool isNull() const { return geohash.length() == 0; }
    const std::string &getGeohashString() const { return geohash; }
    uint64_t getBits() const { return bits; }
    int getPrecision() const { return geohash.length(); }
    const GeographicLib::GeoCoords &getLocation() const {return location; }
    const Bounds &getBounds() const { return bounds; }
    void setGeohash(const std::string &geohash);
    void setLocation(const double &lat, const double &lon);
    void setLocation(const double &lat, const double &lon, unsigned int precision);
    void setLocation(const GeographicLib::GeoCoords &location);
    void setLocation(const GeographicLib::GeoCoords &location, unsigned int precision);
    void setBits(uint64_t bits);
    void setBits(uint64_t bits, unsigned int precision);

    double getDistance(const GeohashLocation &geohashLocation) const;
    double getDistance(const GeographicLib::GeoCoords &location) const;
    bool contains(const GeohashLocation &geohashLocation) const { return bounds.contains(geohashLocation.location); }

    int commonPrefixLength(GeohashLocation &other);

    void getNeighbour(const GeohashLocation::Direction &direction, GeohashLocation &neighbourGeohashLocation) const { GeohashLocation::neighbourGeohashLocation(geohash, direction, neighbourGeohashLocation); }

    static void encode(const GeographicLib::GeoCoords &location, unsigned int precision, std::string &geohash, GeographicLib::GeoCoords &center, Bounds &bounds, uint64_t &bits);
    static void encode(const uint64_t &bits, unsigned int precision, std::string &geohash, GeographicLib::GeoCoords &center, Bounds &bounds);
    static void decode(const std::string &geohash, GeographicLib::GeoCoords &center, Bounds &bounds, uint64_t &bits);
    static void neighbourGeohashLocation(const std::string &geohash, const GeohashLocation::Direction direction, GeohashLocation &neighbourGeohashLocation);
};


} // namespace veins_proj
