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
 * @file RoadNetworkDatabase.cc
 * @author Adrián Juárez Monroy
 */

#include <vector>
#include "veins_proj/roadnetwork/RoadNetworkDatabase.h"
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

using namespace veins_proj;

Define_Module(RoadNetworkDatabase);

/*
 * Destructor.
 */

/*!
 * @brief Eliminar todas las redes viales almacenadas en el diccionario.
 */
RoadNetworkDatabase::~RoadNetworkDatabase() {
    for (auto it = roadNetworksMap.begin(); it != roadNetworksMap.end(); it++)
        delete it->second;
}

/*
 * Interfaz del módulo.
 */

/*!
 * @brief Inicialización.
 */
void RoadNetworkDatabase::initialize() {
    /*
     * Parámetros de configuración.
     */
    databaseDirectory = std::string(par("databaseDirectory").stringValue());
    /*
     * Se buscan todos los archivos XML en el directorio de la base de datos,
     * y se crea una red vial por cada uno.
     */
    boost::filesystem::path databaseDirectoryPath(databaseDirectory);
    ASSERT(
            boost::filesystem::exists(databaseDirectoryPath)
                    && boost::filesystem::is_directory(databaseDirectoryPath));
    boost::filesystem::directory_iterator it(databaseDirectoryPath);
    boost::filesystem::directory_iterator endIt;
    while (it != endIt) {
        const std::string &filePath = it->path().string();
        const std::string &geohash = it->path().stem().string();
        roadNetworksMap[geohash] = new RoadNetwork(geohash, filePath);
        it++;
    }
}

/*
 * Acceso a los atributos.
 */

/*!
 * @brief Acceso a una red vial.
 *
 * @param geohashLocation [in] Región Geohash cuya red vial se obtiene.
 * @return Red vial.
 */
const RoadNetwork* RoadNetworkDatabase::getRoadNetwork(
        const GeohashLocation &geohashLocation) {
    /*
     * Si la longitud del código Geohash es menor a 6, este no es
     * suficientemente precisio como para saber qué red vial se busca.
     */
    if (geohashLocation.getGeohashLength() < 6)
        return nullptr;
    /*
     * Si la longitud del código Geohash es mayor a ogual a 6,
     * se recorta a longitud 6 y se devuelve la red vial correspondiente.
     */
    std::string geohash = geohashLocation.getGeohash().substr(0, 6);
    if (roadNetworksMap.find(geohash) != roadNetworksMap.end())
        return roadNetworksMap[geohash];

    return nullptr;
}
