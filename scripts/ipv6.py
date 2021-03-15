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


def d_to_ipv6_address(d):
    """!
    Convertir un alista de cuatro enteros de 32 bits
    en una dirección IPv6.
    
    @param d: Lista de cuatro números.
    @return: Dirección IPv6
    """
    bits = ''.join(map(lambda x: '{0:032b}'.format(x), d))
    ipv6_address = ipaddress.IPv6Address(int(bits, 2))
    return ipv6_address


if __name__ == '__main__':
    if len(sys.argv) != 5:
        raise ValueError("Proporciona cuatro números.")
    
    d = list(map(int, sys.argv[1:5]))
    ipv6_address = d_to_ipv6_address(d)
    print(ipv6_address)
