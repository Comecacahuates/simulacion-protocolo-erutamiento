#!/usr/bin/env python3
"""!
Crea las direcciones de los nodos de una red en una simulación y
su dirección IPv6, y las guarda en un arhcivo Markdown.

Se utiliza durante la depuración del proyecto
para identificar fácilmente cada nodo con dirección.

Sólo funciona con direcciones que pertenecen a la misma subred.

Uso: listIpv6Addreses.py <Región Geohash> <Número de hosts> <Número de vehículos>
     listIpv6Addreses.py 9g3qxs 2 10

@file: listAddreddes.py
@author: Adrián Juárez Monroy
"""

import enum
import ipaddress
import sys

base32 = '0123456789bcdefghjkmnpqrstuvwxyz'


class NodeType(enum.Enum):
    Host = 0
    Car = 1


def build_ipv6_address(node_type, geohash_region, n, num_hosts):
    """!
    Crea una dirección IPv6.
    
    @param node_type: Tipo de nodo.
    @param geohash_region: Código Geohash de la subred.
    @param n: Número del nodo.
    @param num_hosts: Número de *hosts*
    """
    prefix = '11111100'
    geohash_bits = ''.join(map(
        lambda x: '{0:05b}'.format(base32.index(x)),
        geohash_region))
    d0 = '{0:0<32}'.format(prefix)
    d1 = '{0:0>32}'.format(geohash_bits)
    d2 = '{0:032b}'.format(145359103)
    d3 = '{0:032b}'.format(4261412865 + n if node_type == NodeType.Host else 4261412865 + num_hosts + n)
    ipv6_address = ipaddress.IPv6Address(int(d0 + d1 + d2 + d3, 2))
    return ipv6_address


def save_markdown(markdown_file, hosts_addresses, cars_addresses):
    """!
    Guarda las direcciones en el archivo Markdown.
    
    @param hosts_addresses: Lista de direcciones de los hosts.
    @param cars_addresses: Lista de direcciones de los vehículos.
    """
    with open(markdown_file, 'w') as md_file:
        md_file.write('# *Hosts* #\n\n')
        for i, ipv6_address in enumerate(hosts_addresses):
            bits = '{0:0128b}'.format(int(ipv6_address))
            d = [int(bits[i:i + 32], 2) for i in range(0, 128, 32)]
            md_file.write('### Host[{}] ###\n\n'.format(i))
            md_file.write('IPv6: `{}`\n\n'.format(ipv6_address))
            md_file.write('d: `{}` `{}` `{}` `{}`\n\n'.format(d[0], d[1], d[2], d[3]))
            md_file.write('---\n\n')

        md_file.write('# Vehículos #\n\n')
        for i, ipv6_address in enumerate(cars_addresses):
            bits = '{0:0128b}'.format(int(ipv6_address))
            d = [int(bits[i:i + 32], 2) for i in range(0, 128, 32)]
            md_file.write('### Node[{}] ###\n\n'.format(i))
            md_file.write('IPv6: `{}`\n\n'.format(ipv6_address))
            md_file.write('d: `{}` `{}` `{}` `{}`\n\n'.format(d[0], d[1], d[2], d[3]))
            md_file.write('---\n\n')


if __name__ == '__main__':
    if len(sys.argv) != 4:
        raise ValueError("Proporciona la región Geohas, el número de hosts y el número de vehículos.")

    geohash_region = sys.argv[1]
    num_hosts = int(sys.argv[2])
    num_cars = int(sys.argv[3])

    hosts_addresses = []
    for i in range(num_hosts):
        ipv6_address = build_ipv6_address(NodeType.Host, geohash_region, i, num_hosts)
        hosts_addresses.append(ipv6_address)
        print('Host[{}] -> {}'.format(i, ipv6_address))

    cars_addresses = []
    for i in range(num_cars):
        ipv6_address = build_ipv6_address(NodeType.Car, geohash_region, i, num_hosts)
        cars_addresses.append(ipv6_address)
        print('Node[{}] -> {}'.format(i, ipv6_address))

    save_markdown('ipv6_addresses-{}-{}-{}.md'.format(geohash_region, num_hosts, num_cars), hosts_addresses, cars_addresses)
