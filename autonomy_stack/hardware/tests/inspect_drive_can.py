#!/usr/bin/env python3
"""Regress the diagnostic filter against Linux SocketCAN in an empty netns.

Run only in an isolated Docker container with --network none --cap-add NET_ADMIN.
The test creates its own vcan interface and sends synthetic frames there only.
"""
import importlib.util
from pathlib import Path
import socket
import struct
import threading
from unittest.mock import patch


def attribute(kind, payload):
    size = 4+len(payload)
    return struct.pack('=HH', size, kind)+payload+b'\0'*(-size % 4)


def main():
    # Refuse to transmit if any existing non-loopback interface is present.
    if {name for _, name in socket.if_nameindex()} - {'lo'}:
        raise RuntimeError('Requires an empty network namespace (--network none)')
    payload = struct.pack('=BBHiII', 0, 0, 0, 0, 1, 0xffffffff)
    payload += attribute(3, b'vcan_diag\0')  # IFLA_IFNAME
    payload += attribute(18 | 0x8000, attribute(1, b'vcan\0'))  # IFLA_LINKINFO/KIND
    message = struct.pack('=IHHII', 16+len(payload), 16, 0x605, 1, 0)+payload
    with socket.socket(socket.AF_NETLINK, socket.SOCK_RAW, socket.NETLINK_ROUTE) as rtnl:
        rtnl.bind((0, 0))
        rtnl.settimeout(2)
        rtnl.sendto(message, (0, 0))  # RTM_NEWLINK: create and bring up virtual CAN.
        ack = rtnl.recv(4096)
        error, = struct.unpack_from('=i', ack, 16)
        if error:
            raise RuntimeError(f'Virtual CAN creation failed: {error}')

    helper = Path(__file__).resolve().parents[1] / 'inspect_drive.py'
    spec = importlib.util.spec_from_file_location('inspect_drive', helper)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    data = bytes.fromhex('00010002ffff7fff')
    frame = struct.pack('=IB3x8s', 0x271, 8, data)
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as old, \
            socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sender:
        # Reproduce the original bug using the real kernel receive filter.
        old.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                       struct.pack('=II', 0x200, 0xe0000700))
        old.bind(('vcan_diag',))
        old.settimeout(.05)
        sender.bind(('vcan_diag',))
        sender.send(frame)
        try:
            old.recv(16)
        except socket.timeout:
            pass
        else:
            raise AssertionError('Old filter unexpectedly received a normal frame')

        ready = threading.Event()
        native_socket = socket.socket

        class Receiver(native_socket):
            def bind(self, address):
                super().bind(address)
                ready.set()

        results = []
        worker = threading.Thread(
            target=lambda: results.append(module.capture_feedback('vcan_diag', 1)))
        # Only synchronize bind; filtering, transmission and reception use real sockets.
        with patch.object(module.socket, 'socket', Receiver):
            worker.start()
            if not ready.wait(timeout=2):
                worker.join(timeout=2)
                raise AssertionError(f'Capture never bound its socket: {results}')
            for _ in range(5):
                sender.send(frame)
                sender.send(struct.pack('=IB3x8s', 0x371, 8, b'\0'*8))
                sender.send(struct.pack('=IB3x8s', socket.CAN_EFF_FLAG | 0x271, 8, b'\0'*8))
                sender.send(struct.pack('=IB3x8s', socket.CAN_RTR_FLAG | 0x271, 8, b'\0'*8))
            worker.join(timeout=3)
        assert not worker.is_alive(), 'Capture failed to stop'
    result = results[0]
    assert result['error'] is None, result
    assert list(result['frames']) == ['0x271'], result
    assert result['frames']['0x271']['count'] == 5, result
    assert result['frames']['0x271']['examples_hex'] == [data.hex()], result
    print('PASS: old filter loses normal feedback; corrected capture receives all five '
          'frames and excludes unrelated, extended and RTR frames.')


if __name__ == '__main__':
    main()
