"""Create a virtual CAN interface only inside the isolated Docker test network."""
import os
import socket
import struct


def create():
    assert os.environ.get('RANGER_ISOLATED_CAN_TEST') == '1'
    assert {name for _, name in socket.if_nameindex()} <= {'lo', 'eth0'}

    def attribute(kind, payload):
        size = 4 + len(payload)
        return struct.pack('=HH', size, kind) + payload + b'\0' * (-size % 4)

    payload = struct.pack('=BBHiII', 0, 0, 0, 0, 1, 0xffffffff) + attribute(3, b'vcan_nav\0')
    payload += attribute(18 | 0x8000, attribute(1, b'vcan\0'))
    with socket.socket(socket.AF_NETLINK, socket.SOCK_RAW, socket.NETLINK_ROUTE) as link:
        link.bind((0, 0)); link.settimeout(2)
        link.sendto(struct.pack('=IHHII', 16 + len(payload), 16, 0x605, 1, 0) + payload, (0, 0))
        assert struct.unpack_from('=i', link.recv(4096), 16)[0] == 0, 'Virtual CAN creation failed'
    sender = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sender.bind(('vcan_nav',))
    return sender
