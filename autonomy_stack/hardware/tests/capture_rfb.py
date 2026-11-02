"""Capture the test container's localhost-only virtual display as a PNG."""
import socket
import struct
import zlib
from pathlib import Path


def read(connection,size):
    data=bytearray()
    while len(data)<size:
        part=connection.recv(size-len(data))
        if not part:raise EOFError('VNC closed the connection')
        data.extend(part)
    return bytes(data)


def chunk(kind,data):
    return struct.pack('!I',len(data))+kind+data+struct.pack('!I',zlib.crc32(kind+data)&0xffffffff)


with socket.create_connection(('127.0.0.1',15900),timeout=10) as connection:
    assert read(connection,12).startswith(b'RFB 003.')
    connection.sendall(b'RFB 003.008\n')
    count=read(connection,1)[0]
    assert 1 in read(connection,count),'Test VNC did not offer localhost no-auth mode'
    connection.sendall(b'\x01')
    assert read(connection,4)==b'\0\0\0\0'
    connection.sendall(b'\x01')
    width,height=struct.unpack('!HH',read(connection,4))
    read(connection,16)
    name_size,=struct.unpack('!I',read(connection,4));read(connection,name_size)
    pixel_format=struct.pack('!BBBBHHHBBB3x',32,24,0,1,255,255,255,16,8,0)
    connection.sendall(b'\0\0\0\0'+pixel_format)
    connection.sendall(struct.pack('!BBHi',2,0,1,0))  # Raw encoding only.
    connection.sendall(struct.pack('!BBHHHH',3,0,0,0,width,height))
    assert read(connection,1)==b'\0','Expected framebuffer update'
    read(connection,1)
    rectangles,=struct.unpack('!H',read(connection,2))
    image=bytearray(width*height*3)
    for _ in range(rectangles):
        x,y,w,h,encoding=struct.unpack('!HHHHi',read(connection,12))
        assert encoding==0,encoding
        pixels=read(connection,w*h*4)
        for row in range(h):
            for column in range(w):
                source=(row*w+column)*4;target=((y+row)*width+x+column)*3
                image[target:target+3]=pixels[source:source+3][::-1]
    raw=b''.join(b'\0'+image[row*width*3:(row+1)*width*3] for row in range(height))
    png=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!IIBBBBB',width,height,8,2,0,0,0))
    png+=chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b'')
    Path('/tmp/rviz.png').write_bytes(png)
    print(f'Captured {width}x{height} RViz display')
