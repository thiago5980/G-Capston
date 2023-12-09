import serial
import os
from time import sleep

class PacketHandler:
  def __init__(self, _port_name, _baud_rate):
    self.port_name = _port_name
    self.baud_rate = _baud_rate
    self._ser = serial.Serial(self.port_name, self.baud_rate, timeout=0.001)
    self._ser.reset_input_buffer()
    self._ser.reset_output_buffer()
    
  def get_port_state(self):
    return self._ser.is_open

  def close_port(self):
    print("Close Port")
    self._ser.close()
    
  def write_motor(self, cleaner, brush, water):
    header_1 = b'\xff'
    if cleaner:
      byte_cleaner = b'\x01'[0]
    else:
      byte_cleaner = b'\x00'[0]
    if brush:
      byte_brush = b'\x02'[0]
    else:
      byte_brush = b'\x00'[0]
    if water:
      byte_water = b'\x04'[0]
    else:
      byte_water = b'\x00'[0]

    tail = b'\xEE'
    
    sum = byte_cleaner | byte_brush | byte_water
    send_data = header_1 + bytes([sum]) + tail
    print("get_topic")
    self._ser.write(send_data)