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
 * @file ExpiringValuesMap.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include <omnetpp.h>
#include <map>
#include <utility>

namespace veins_proj {

//! Valor del diccionario.
template<typename V>
struct ExpiringValue {
    //! Hora de expiración del valor.
    omnetpp::simtime_t expiryTime;
    //! Valor.
    V value;
};

//! Clase que implementa un diccionario en el que los valores tienen una hora
//! de expiración.
template<typename K, typename V>
class ExpiringValuesMap {

public:

    //! Valor
    typedef ExpiringValue<V> MapValue;
    //! Diccionario.
    typedef std::map<K, MapValue> Map;
    //! Registro del diccionario.
    typedef std::pair<K, MapValue> MapValueType;
    //! Iterador de registros para diccionario.
    typedef typename Map::iterator Iterator;
    //! Iterador de registros para diccionario constante.
    typedef typename Map::const_iterator ConstIterator;

protected:

    //! Diccionario.
    Map map;

public:
    //! Obtener diccionario.
    Map& getMap() {
        return map;
    }
    //! Obtener diccionario.
    const Map& getMap() const {
        return map;
    }

    //! Eliminar valores cuya hora de expiración es anterior a la hora indicada.
    void removeOldValues(omnetpp::simtime_t time) {
        Iterator it = map.begin();
        Iterator endIt = map.end();

        while (it != endIt)
            if (it->second.expiryTime <= time)
                map.erase(it++);

            else
                it++;
    }

    //! Obtener la hora de expiración más próxima.
    omnetpp::simtime_t getNextExpiryTime() const {
        omnetpp::simtime_t nextExpiryTime = omnetpp::SimTime::getMaxTime();

        ConstIterator it = map.begin();
        ConstIterator endIt = map.end();
        while (it != endIt) {
            if (nextExpiryTime > it->second.expiryTime)
                nextExpiryTime = it->second.expiryTime;

            it++;
        }

        return nextExpiryTime;
    }
};

//! Clase que implementa un multidiccionario en el que los valores tienen
//! una hora de expiración.
template<typename K, typename V>
class ExpiringValuesMultimap {

public:

    //! Valor
    typedef ExpiringValue<V> MultimapValue;
    //! Diccionario.
    typedef std::map<K, MultimapValue> Multimap;
    //! Registro del diccionario.
    typedef std::pair<K, MultimapValue> MultimapValueType;
    //! Iterador de registros para diccionario.
    typedef typename Multimap::iterator Iterator;
    //! Iterador de registros para diccionario constante.
    typedef typename Multimap::const_iterator ConstIterator;

protected:

    //! Diccionario.
    Multimap multimap;

public:
    //! Obtener diccionario.
    Multimap& getMultimap() {
        return multimap;
    }
    //! Obtener diccionario.
    const Multimap& getMultimap() const {
        return multimap;
    }

    //! Eliminar valores cuya hora de expiración es anterior a la hora indicada.
    void removeOldValues(omnetpp::simtime_t time) {
        Iterator it = multimap.begin();

        while (it != multimap.end())
            if (it->second.expiryTime <= time)
                multimap.erase(it++);

            else
                it++;
    }

    //! Obtener la hora de expiración más próxima.
    omnetpp::simtime_t getNextExpiryTime() const {
        omnetpp::simtime_t nextExpiryTime = omnetpp::SimTime::getMaxTime();

        ConstIterator it = multimap.begin();
        ConstIterator endIt = multimap.end();
        while (it != endIt) {
            if (nextExpiryTime > it->second.expiryTime)
                nextExpiryTime = it->second.expiryTime;
            it++;
        }

        return nextExpiryTime;
    }
};

}    // namespace veins_proj
