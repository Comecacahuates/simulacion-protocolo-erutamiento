/*
 * GeohashLocation.cpp
 *
 *  Created on: Jun 9, 2020
 *      Author: adrian
 */

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/Geodesic.hpp>
#include "veins_proj/geohash/GeohashLocation.h"


using namespace veins_proj;


const std::string GeohashLocation::base32 = "0123456789bcdefghjkmnpqrstuvwxyz";

const std::string GeohashLocation::neighbour[4][2] = {
        { "p0r21436x8zb9dcf5h7kjnmqesgutwvy", "bc01fg45238967deuvhjyznpkmstqrwx" },
        { "bc01fg45238967deuvhjyznpkmstqrwx", "p0r21436x8zb9dcf5h7kjnmqesgutwvy" },
        { "14365h7k9dcfesgujnmqp0r2twvyx8zb", "238967debc01fg45kmstqrwxuvhjyznp" },
        { "238967debc01fg45kmstqrwxuvhjyznp", "14365h7k9dcfesgujnmqp0r2twvyx8zb" }
};

const std::string GeohashLocation::border[4][2] = {
        { "prxz",     "bcfguvyz" },
        { "bcfguvyz", "prxz" },
        { "028b",     "0145hjnp" },
        { "0145hjnp", "028b" }
};


GeohashLocation::GeohashLocation():
        location(),
        bounds() {
    geohash = "";
    bits = 0;
}


GeohashLocation::GeohashLocation(const std::string &geohash) {
    setGeohash(geohash);
}


GeohashLocation::GeohashLocation(const GeographicLib::GeoCoords &location, unsigned int precision) {
    setLocation(location, precision);
}


GeohashLocation::GeohashLocation(uint64_t bits, unsigned int precision) {
    setBits(bits, precision);
}


GeohashLocation::~GeohashLocation() {}


void GeohashLocation::setGeohash(const std::string &geohash) {
    this->geohash = geohash;
    GeohashLocation::decode(geohash, location, bounds, bits);
}


void GeohashLocation::setLocation(const double &lat, const double &lon) {
    GeohashLocation::encode(GeographicLib::GeoCoords(lat, lon), geohash.length(), geohash, location, bounds, bits);
    this->location.Reset(lat, lon);
}


void GeohashLocation::setLocation(const double &lat, const double &lon, unsigned int precision) {
    GeohashLocation::encode(GeographicLib::GeoCoords(lat, lon), precision, geohash, location, bounds, bits);
    this->location.Reset(lat, lon);
}


void GeohashLocation::setLocation(const GeographicLib::GeoCoords &location) {
    GeohashLocation::encode(location, geohash.length(), geohash, this->location, bounds, bits);
    this->location = location;
}


void GeohashLocation::setLocation(const GeographicLib::GeoCoords &location, unsigned int precision) {
    GeohashLocation::encode(location, precision, geohash, this->location, bounds, bits);
    this->location = location;
}


void GeohashLocation::setBits(uint64_t bits) {
    this->bits = bits;
    GeohashLocation::encode(bits, geohash.length(), geohash, location, bounds);
}


void GeohashLocation::setBits(uint64_t bits, unsigned int precision) {
    this->bits = bits;
    GeohashLocation::encode(bits, precision, geohash, location, bounds);
}


double GeohashLocation::getDistance(const GeohashLocation &geohashLocation) const {
    return getDistance(geohashLocation.getLocation());
}


double GeohashLocation::getDistance(const GeographicLib::GeoCoords &location) const {
    const GeographicLib::Geodesic &geod = GeographicLib::Geodesic::WGS84();

    double distance;

    geod.Inverse(this->location.Latitude(), this->location.Longitude(), location.Latitude(), location.Longitude(), distance);

    return distance;
}


int GeohashLocation::commonPrefixLength(GeohashLocation &other) {
    uint64_t bits1 = bits >> 4,
             bits2 = other.bits >> 4;

    int commonPrefixLength = 60;

    for (uint64_t diff = bits1 ^ bits2; diff != 0; commonPrefixLength--)
        diff >>= 1;

    return commonPrefixLength;
}


void GeohashLocation::encode(const GeographicLib::GeoCoords &location, unsigned int precision, std::string &geohash, GeographicLib::GeoCoords &center, Bounds &bounds, uint64_t &bits) {
    precision = precision < 1 || precision > 12 ? 12 : precision;

    geohash.clear();

    bool evenBit = true;
    unsigned int idx = 0;
    unsigned int bit = 0;

    double lonMid,
           lonMin = -180,
           lonMax =  180,
           latMid,
           latMin =  -90,
           latMax =   90;

    bits = 0;
    uint64_t mask = 1; mask <<= 63;

    while (geohash.length() < precision) {
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

    center = GeographicLib::GeoCoords((latMin + latMax) / 2.0, (lonMin + lonMax) / 2.0);
    bounds = Bounds(latMax, lonMax, latMin, lonMin);
}


void GeohashLocation::encode(const uint64_t &bits, unsigned int precision, std::string &geohash, GeographicLib::GeoCoords &center, Bounds &bounds) {
    precision = precision < 1 || precision > 12 ? 12 : precision;

    geohash.clear();

    bool evenBit = true;
    unsigned int idx = 0;
    uint64_t bit = 0;

    double lonMid,
           lonMin = -180,
           lonMax =  180,
           latMid,
           latMin =  -90,
           latMax =   90;

    uint64_t mask = 1; mask <<= 63;

    int nBits = precision * 5;

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

    center = GeographicLib::GeoCoords((latMin + latMax) / 2.0, (lonMin + lonMax) / 2.0);
    bounds = Bounds(latMax, lonMax, latMin, lonMin);
}


void GeohashLocation::decode(const std::string &geohash, GeographicLib::GeoCoords &center, Bounds &bounds, uint64_t &bits) {
    bool evenBit = true;
    unsigned int idx = 0;
    uint8_t bit = 0;

    double lonMid,
           lonMin = -180,
           lonMax =  180,
           latMid,
           latMin =  -90,
           latMax =   90;

    bits = 0;
    uint64_t mask = 1; mask <<= 63;

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

    center = GeographicLib::GeoCoords((latMin + latMax) / 2.0, (lonMin + lonMax) / 2.0);
    bounds = Bounds(latMax, lonMax, latMin, lonMin);
}


void GeohashLocation::neighbourGeohashLocation(const std::string &geohash, const GeohashLocation::Direction direction, GeohashLocation &neighbourGeohashLocation) {
    char lastChar = geohash.back();
    std::string parent = geohash.substr(0, geohash.length() - 1);

    unsigned int type = geohash.length() % 2;

    if (GeohashLocation::border[direction][type].find(lastChar) != std::string::npos && parent.empty())
        GeohashLocation::neighbourGeohashLocation(parent, direction, neighbourGeohashLocation);

    parent.push_back(GeohashLocation::base32.at(GeohashLocation::neighbour[direction][type].find(lastChar)));
    neighbourGeohashLocation.setGeohash(parent);
}
