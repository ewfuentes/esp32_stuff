import socket
import struct
import sys
sys.path.append('/home/erick/code/esp8266/esp32_stuff/app/udp_comm/proto-py')
import imu_samples_pb2

MCAST_GRP = '239.1.2.4'
MCAST_PORT = 23456
IS_ALL_GROUPS = True

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
if IS_ALL_GROUPS:
    # on this port, receives ALL multicast groups
    sock.bind(('', MCAST_PORT))
else:
    # on this port, listen ONLY to MCAST_GRP
    sock.bind((MCAST_GRP, MCAST_PORT))
#mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
#
#sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

while True:
  # For Python 3, change next line to "print(sock.recv(10240))"
  data = sock.recv(10240)
  imu_samples = imu_samples_pb2.ImuSamples()
  imu_samples.ParseFromString(data)

  print(f't: {imu_samples.time_of_validity_us / 1e6} ax: {imu_samples.accel_x_mpss[0]} ay: {imu_samples.accel_y_mpss[0]}, az: {imu_samples.accel_z_mpss[0]}')
