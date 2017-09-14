#!usr/bin/python3

from __future__ import print_function
import smbus
import time

def help():
  Robot().help()

class Robot:

  def __init__(self):
    self.bus = smbus.SMBus(1)
    self.restore_defaults()

  def restore_defaults(self):
    self.buzzer(0)
    self.red(0)
    self.green(0)
    self.blue(0)
    self.sleep(1)
    self.motor_a(0)
    self.motor_b(0)
    self.speed_a(1)
    self.speed_b(1)
    self.motor_a_direction(1)
    self.motor_b_direction(1)
    self.microstepping(16)
    self.power(3)
    

  def help(self):
    print("Available commands:")
    print(" red(), green(), blue()")
    print(" buzzer()")
    print(" status()")
    print(" Examples: green(1), red('on'), blue('Off')\n")

  def write(self, data_byte):
    self.bus.write_byte_data(0x70, 0, data_byte)
    print("Byte {0} sent".format(data_byte))

  def write_block(self, data_block):
    self.bus.write_i2c_block_data(0x70, 0, data_block)
    print("{0} sent".format(data_block))
  
  def read(self, number_of_bytes):
    vals = self.bus.read_i2c_block_data(0x70, 0x00)
    return vals

  def green(self, on):
    try:
      if on.upper() in ('OFF', 'AUS'):
        on = False
      else:
        on = True
    except:
      pass
    if on:
      self.write(0x51)
    else:
      self.write(0x50)

  def red(self, on):
    try:
      if on.upper() in ('OFF', 'AUS'):
        on = False
      else:
        on = True
    except:
      pass
    if on:
      self.write(0x53)
    else:
      self.write(0x52)

  def blue(self, on):
    try:
      if on.upper() in ('OFF', 'AUS'):
        on = False
      else:
        on = True
    except:
      pass
    if on:
      self.write(0x55)
    else:
      self.write(0x54)

  def buzzer(self, on):
    try:
      if on.upper() in ('OFF', 'AUS'):
        on = False
      else:
        on = True
    except:
      pass
    if on:
      self.write(0x57)
    else:
      self.write(0x56)

  def status(self):
    data = self.read(5)
    if data[1] == 0:
      print('Green is off')
    else:
      print('Green is on')
      
    if data[2] == 0:
      print('Red is off')
    else:
      print('Red is on')
      
    if data[3] == 0:
      print('Blue is off')
    else:
      print('Blue is on')
      
    if data[4] == 0:
      print('Buzzer is off')
    else:
      print('Buzzer is on')

  def enable(self, on):
    if on:
      self.write(0x59)
    else:
      self.write(0x58)

  def reset(self, on):
    if on:
      self.write(0x61)
    else:
      self.write(0x60)

  def sleep(self, on):
    if on:
      self.write(0x63)
    else:
      self.write(0x62)

  def motor_a(self, on):
    if on:
      self.write(0x65)
    else:
      self.write(0x64)

  def motor_b(self, on):
    if on:
      self.write(0x67)
    else:
      self.write(0x66)

  def microstepping(self, stepsize):
    if stepsize==1:
      self.write(0x68)
    elif stepsize==2:
      self.write(0x69)
    elif stepsize==4:
      self.write(0x6A)
    elif stepsize==16:
      self.write(0x6B)

  def motor_a_direction(self, direction):
    if direction:
      self.write(0x6C)
    else:
      self.write(0x6D)

  def motor_b_direction(self, direction):
    if direction:
      self.write(0x6E)
    else:
      self.write(0x6F)

  def speed_a(self, speed):
    self.write_block([0x10, speed])

  def speed_b(self, speed):
    self.write_block([0x11, speed])

  def power(self, power):
    self.write_block([0x12, power])

  def drive(self, millimeters, speed=0):
    msb = abs(millimeters) // 256
    lsb = abs(millimeters) % 256
    if speed in [1,2,3,4,5]:
      self.speed_a(speed)
      self.speed_b(speed)
    if millimeters>0:
      self.motor_a_direction(1)
      self.motor_b_direction(1)
    else:
      self.motor_a_direction(0)
      self.motor_b_direction(0)
    self.write_block([0x22, msb, lsb, msb, lsb])


  def turn(self, angle, speed=0):
    width = 162.0 #width in millimeters
    circle = width * 3.1415
    multiplier = circle / 360.0
    milimeters = int(multiplier*angle)
    msb = abs(milimeters) // 256
    lsb = abs(milimeters) % 256
    if speed in [1,2,3,4,5]:
      self.speed_a(speed)
      self.speed_b(speed)
    self.speed_b(speed)
    if angle>0:
      self.motor_a_direction(0)
      self.motor_b_direction(1)
    else:
      self.motor_a_direction(1)
      self.motor_b_direction(0)
    self.write_block([0x22, msb, lsb, msb, lsb])


  def demo(self):
    self.restore_defaults()
    #flash red
    self.red(1)
    time.sleep(1)
    self.red(0)
    #flash green
    self.green(1)
    time.sleep(1)
    self.green(0)
    #flash blue
    self.blue(1)
    time.sleep(1)
    self.blue(0)
    #beep
    self.buzzer(1)
    time.sleep(1)
    self.buzzer(0)
    #increase power
    self.power(6)
    #drive
    self.drive(100,1)
    time.sleep(4)
    #turn
    self.turn(180,1)
    time.sleep(9)
    #drive
    self.drive(200,3)
    time.sleep(4)
    #turn again
    self.turn(-90,1)
    time.sleep(6)
    self.turn(180,4)
    time.sleep(5)
    #drive
    self.drive(200,5)
    time.sleep(2)
    #turn
    self.turn(-45,2)
    time.sleep(3)
    #beep twice
    self.buzzer(1)
    self.buzzer(0)
    self.buzzer(1)
    self.buzzer(0)
    #decrease power
    self.power(3)
