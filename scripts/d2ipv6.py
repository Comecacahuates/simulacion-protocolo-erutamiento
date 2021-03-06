#!/usr/bin/env python3
"""!
Convierte una secuencua de cuatro enteros de 32 bits
a una dirección IPv6, y muestra el resultado.

Se utiliza para hacer legibles las direcciones IPv6,
ya que en la depuración se muestran como un arreglo de
cuatro números enteros (que pueden ir separados por comas).

Uso: d2ipv6.py <d0> <d1> <d2> <d3>
     d2ipv6.py 4273995776, 317840312, 145359103, 4261412865

@file: d2ipv6.py
@author: Adrián Juárez Monroy
"""

import ipaddress
import re
import sys


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

    d = list(map(lambda x: int(re.sub("[^\d\.]", "", x)), sys.argv[1:5]))
    ipv6_address = d_to_ipv6_address(d)
    print(ipv6_address)
