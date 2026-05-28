import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", 4210))

while True:
    data, addr = sock.recvfrom(64)
    distance, angle = data.decode().split(",")
    print(f"Distance: {distance} cm, Angle: {angle} deg")