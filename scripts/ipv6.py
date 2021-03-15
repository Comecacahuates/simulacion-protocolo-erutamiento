#!/usr/bin/python3
"""!
Convierte una seciencua de cuatro enteros de 32 bits
a una dirección IPv6, y muestra el resultado.

Se utiliza para hacer legibles las direcciones IPv6,
ya que en la depuración se muestran como un arreglo de
cuatro números enteros.
"""

import sys
import ipaddress


def d_to_ipv6(d):
    """!
    @brief Convertir un alista de cuatro enteros de 32 bits
    en una dirección IPv6.
    
    @param d: Lista de cuatro números.
    @return: Dirección IPv6
    """
    bits = ''.join(map(lambda x: '{0:032b}'.format(x), d))
    octetos = []
    for i in range(len(bits) // 16):
        octetos.append('{0:0>4x}'.format(int(bits[i:i + 16], 2)))
    ipv6 = ipaddress.IPv6Address(int(''.join(octetos), 16))
    return ipv6


if __name__ == '__main__':
    if len(sys.argv) != 5:
        raise ValueError("Proporciona cuatro números.")
    
    d = list(map(int, sys.argv[1:5]))
    ipv6 = d_to_ipv6(d)
    print(ipv6)
