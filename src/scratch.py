"""
scratch

Reads control commands wirelessly over bluetooth from a Sony DualShock 3
PlayStation controller. Interprets commands, and converts them into driving
instructions for balancing robot which are written out via serial port to Arduino

Luke Arend
July 2015
Bethel University NanoLab
"""


######## Includes ########
import pygame
import time
import serial
import subprocess
import picamera
import datetime
import sys
import os



######## Global Variable Declarations ########
#### Controller state flag dictionaries ####
buttonValue = {'select': 0,
               'start': 0,
               'clickL': 0,
               'clickR': 0,
               'dPadUp': 0,
               'dPadRight': 0,
               'dPadDown': 0,
               'dPadLeft': 0,
               'triggerL': 0,
               'triggerR': 0,
               'bumperL': 0,
               'bumperR': 0,
               'triangle': 0,
               'circle': 0,
               'cross': 0,
               'square': 0,
               'sum': 0}

axisValue = {'xAxisL': 0,
             'yAxisL': 0,
             'xAxisR': 0,
             'yAxisR': 0,
             'clickL': 0,
             'clickR': 0,
             'dPadUp': 0,
             'dPadRight': 0,
             'dPadDown': 0,
             'dPadLeft': 0,
             'triggerL': 0,
             'triggerR': 0,
             'bumperL': 0,
             'bumperR': 0,
             'triangle': 0,
             'circle': 0,
             'cross': 0,
             'square': 0}

lastButtonValue = {'dPadUp': 0,
                   'dPadRight': 0,
                   'dPadDown': 0,
                   'dPadLeft': 0,
                   'bumperL': 0,
                   'bumperR': 0,
                   'start': 0,
                   'triangle': 0,
                   'circle': 0,
                   'cross': 0,
                   'square': 0,
                   'sum': 0}

controllerOptions = {'mapping': True, #True is turn/drive, False is left/right
                     'invertMult': 1,
                     'powerMult': .5,
                     'smoothing': 1,
                     'autonomous': False,
                     'previewing': False}



######## Initialization ########
print("Launching initialization...")
print()
#### Initialize pygame ####
print("Initializing pygame...")
pygame.init()
print()


#### Initialize controller ####
print("Initializing controller connection...")
joystickAvailable = pygame.joystick.get_count() # check if joystick is available
while(joystickAvailable == 0): # if not, try re-launching sixad
  print("Controller connection failed.")
  time.sleep(1)
  print("Configuring DualShock 3 controller...")
  time.sleep(1)
  sixad = subprocess.Popen(["/usr/bin/sixad", "--stop"],
                                     universal_newlines=True)
  sixad = subprocess.Popen(["/usr/bin/sixad", "--start"],
                           universal_newlines=True)
  timeout = 10  # timeout for connection loop in seconds
  startTime = time.time()
  print("Trying to connect (please press PS3 button)...")
  while(time.time() - startTime < timeout):
    pygame.joystick.quit()
    pygame.joystick.init()
    joystickAvailable = pygame.joystick.get_count()
    if(joystickAvailable != 0):
      break
  if(joystickAvailable != 0):
      break
  else:
    print("Connection process timed out.")
    time.sleep(1)
print("Controller found!")
print()
  
print("Initializing controller instance...")
dualshock = pygame.joystick.Joystick(0) # create joystick instance for DualShock
dualshock.init()

print("Initialized joystick: '%s'" % dualshock.get_name())    # print some useful info
print("Number of axes: %s" % dualshock.get_numaxes())
print("Number of trackballs: %s" % dualshock.get_numballs())
print("Number of buttons: %s" % dualshock.get_numbuttons())
print("Number of hats: %s" % dualshock.get_numhats())
print()


#### Initialize serial port ####
def initializeSerial(portName): # recursive function to attempt to open serial port
  try:
    port = serial.Serial(portName, baudrate=115200, timeout=.005)
  except OSError:
    print("Port '%s' not found." % portName)
    time.sleep(5)
    """print("Please check cable connection, then press 'start' to try again.")
    while(True):
      events = pygame.event.get()   
      for event in events:
        if(event.type == 10 and event.button == 3):
          break
      else:
        continue # if for loop not broken, continue
      break"""
    print("Re-attempting to initialize serial port '%s'..." % portName)
    port = initializeSerial(portName)
  return port

portName = '/dev/ttyAMA0'
#portName = '/dev/ttyACM0'
print("Initializing serial port '%s'..." % portName)
"""print("Press 'start' to open serial connection.")
while(True):
  events = pygame.event.get()   
  for event in events:
    if(event.type == 10 and event.button == 3):
      break
  else:
    continue # if for loop not broken, continue
  break"""
port = initializeSerial(portName)
port.flushInput()
port.flushOutput()
#time.sleep(1)
print("Serial port '%s' opened successfully." % portName)
print()


#### Initialize PiCamera ####
print("Initializing camera...")
try:
  camera = picamera.PiCamera()
  camera.resolution = (1024, 768)
except picamera.exc.PiCameraError:
  print("Camera is not enabled.")
print()

print("Initialization complete!")
print()



######## Main code ########
#### Controller functions ####
def readController():
  events = pygame.event.get() # read events from the controller
    
  buttonValue['select'] = dualshock.get_button(0) # get button states
  buttonValue['clickL'] = dualshock.get_button(1) # they are stored in flags
  buttonValue['clickR'] = dualshock.get_button(2)
  buttonValue['start'] = dualshock.get_button(3)
  buttonValue['dPadUp'] = dualshock.get_button(4)
  buttonValue['dPadRight'] = dualshock.get_button(5)
  buttonValue['dPadDown'] = dualshock.get_button(6)
  buttonValue['dPadLeft'] = dualshock.get_button(7)
  buttonValue['triggerL'] = dualshock.get_button(8)
  buttonValue['triggerR'] = dualshock.get_button(9)
  buttonValue['bumperL'] = dualshock.get_button(10)
  buttonValue['bumperR'] = dualshock.get_button(11)
  buttonValue['triangle'] = dualshock.get_button(12)
  buttonValue['circle'] = dualshock.get_button(13)
  buttonValue['cross'] = dualshock.get_button(14)
  buttonValue['square'] = dualshock.get_button(15)
    
  axisValue['xAxisL'] = dualshock.get_axis(0) # get axis states
  axisValue['yAxisL'] = dualshock.get_axis(1) # they are stored in flags
  axisValue['xAxisR'] = dualshock.get_axis(2)
  axisValue['yAxisR'] = dualshock.get_axis(3)
  axisValue['dPadUpPressure'] = dualshock.get_axis(8)
  axisValue['dPadRightPressure'] = dualshock.get_axis(9)
  axisValue['dPadDownPressure'] = dualshock.get_axis(10)
  axisValue['dPadLeftPressure'] = dualshock.get_axis(11)
  axisValue['triggerLPressure'] = dualshock.get_axis(12)
  axisValue['triggerRPressure'] = dualshock.get_axis(13)
  axisValue['bumperLPressure'] = dualshock.get_axis(14)
  axisValue['bumperRPressure'] = dualshock.get_axis(15)
  axisValue['trianglePressure'] = dualshock.get_axis(16)
  axisValue['circlePressure'] = dualshock.get_axis(17)
  axisValue['crossPressure'] = dualshock.get_axis(18)
  axisValue['squarePressure'] = dualshock.get_axis(19)

  buttonValue['sum'] = axisValue['xAxisL'] + axisValue['yAxisL'] + axisValue['xAxisR'] + axisValue['yAxisR']

def toggleController(): # allows controller settings to be toggled
  if((buttonValue['dPadUp'] == 1) & (lastButtonValue['dPadUp'] == 0)):
    controllerOptions['powerMult'] += .25
    controllerOptions['powerMult'] = min(1, controllerOptions['powerMult'])
  if((buttonValue['dPadDown'] == 1) & (lastButtonValue['dPadDown'] == 0)):
    controllerOptions['powerMult'] -= .25
    controllerOptions['powerMult'] = max(0, controllerOptions['powerMult'])
  if((buttonValue['dPadRight'] == 1) & (lastButtonValue['dPadRight'] == 0)):
    controllerOptions['smoothing'] += .1
    controllerOptions['smoothing'] = min(1, controllerOptions['smoothing'])
  if((buttonValue['dPadLeft'] == 1) & (lastButtonValue['dPadLeft'] == 0)):
    controllerOptions['smoothing'] -= .1
    controllerOptions['smoothing'] = max(0, controllerOptions['smoothing'])
  if((buttonValue['bumperL'] == 1) & (lastButtonValue['bumperL'] == 0)):
    controllerOptions['mapping'] = not controllerOptions['mapping']
  if((buttonValue['start'] == 1) & (lastButtonValue['start'] == 0)):
    controllerOptions['autonomous'] = not controllerOptions['autonomous']
  if(buttonValue['sum'] != lastButtonValue['sum']):
    controllerOptions['autonomous'] = False
    #^ if the joysticks have been moved, autonomous mode is manually overridden
  if((buttonValue['triangle'] == 1) & (lastButtonValue['triangle'] == 0)):
    controllerOptions['previewing'] = not controllerOptions['previewing']
    if(controllerOptions['previewing'] == True):
      try:
        #camera.start_preview()
        print("Camera preview started.")
      except:
        print("Failed to start camera preview.")
    else:
      try:
        #camera.stop_preview()
        print("Camera preview stopped.")
      except:
        print("Failed to stop camera preview.")
  if((buttonValue['square'] == 1) & (lastButtonValue['square'] == 0)):
    try:    
      now = datetime.datetime.now()
      microsecond = now.microsecond
      second = now.second
      minute = now.minute
      hour = now.hour
      day = now.day
      month = now.month
      year = now.year
      timestamp = "{0:04d}-{1:02d}-{2:02d}T{3:02d}:{4:02d}:{5:02d}.{6:06d}".format(year, month, day, hour, minute, second, microsecond)
      filePath = "Pictures/picamera/flexibot/" + timestamp + ".jpg"
      #camera.capture(filePath)
      print('Image captured with timestamp %s.' % timestamp)
    except:
      print("Failed to capture image.")
    
def updateLastValues(): 
  lastButtonValue['dPadUp'] = buttonValue['dPadUp']
  lastButtonValue['dPadRight'] = buttonValue['dPadRight']
  lastButtonValue['dPadDown'] = buttonValue['dPadDown']
  lastButtonValue['dPadLeft'] = buttonValue['dPadLeft']
  lastButtonValue['bumperL'] = buttonValue['bumperL']
  lastButtonValue['bumperR'] = buttonValue['bumperR']
  lastButtonValue['start'] = buttonValue['start']
  lastButtonValue['triangle'] = buttonValue['triangle']
  lastButtonValue['circle'] = buttonValue['circle']
  lastButtonValue['cross'] = buttonValue['cross']
  lastButtonValue['square'] = buttonValue['square']
  lastButtonValue['sum'] = buttonValue['sum'] 

def serialRead(port): # reads serial data from Arduino
  try:
    throwawayString = port.readline()
    inputString = port.readline().decode('UTF-8')
    port.flushInput()
  except UnicodeDecodeError:
    inputString = ""

  if(inputString == ""):  # Arduino not connected
    theta = 0
    proxFL = 0
    proxFR = 0
    proxBL = 0
    proxBR = 0
    return theta, proxFL, proxFR, proxBL, proxBR

  try:
    thetaString, proxFLstring, proxFRstring, proxBLstring, proxBRstring = inputString.split(":", 5 - 1)
  #^ string is split 3 times, yielding 4 data points
  except ValueError:
  #^ an occasional serial communication error from the Arduino can occur;
  #^ this tries one more time in hopes of getting a good read and continuing
    throwawayString = port.readline()
    inputString = port.readline().decode('UTF-8')
    port.flushInput()
    thetaString, proxFLstring, proxFRstring, proxBLstring, proxBRstring = inputString.split(":", 5 - 1)
  theta = float(thetaString)
  proxFL = float(proxFLstring)
  proxFR = float(proxFRstring)
  proxBL = float(proxBLstring)
  proxBR = float(proxBRstring)
  return theta, proxFL, proxFR, proxBL, proxBR

def autonomousDrive(proxFL, proxFR, proxBL, proxBR): # drives the robot in autonomous setting
  fearThreshold = 30  # number of cm for it to start steering away from obstacle
  stopThreshold = 20 # number of cm causing it to stop

  maxSpeed = .15

  controllerOptions['smoothing'] == 0

  if(proxFL == 0 and proxFR == 0):  # if no Arduino connection
    driveSpeed = 0
    turnSpeed = 0  
  elif(proxFL < fearThreshold and proxFR < fearThreshold):  # if object detected head on
    if(proxFL < proxFR):
      driveSpeed = max(0, (proxFL - stopThreshold)/(fearThreshold - stopThreshold))
      turnSpeed = min(1, (fearThreshold - proxFL)/(fearThreshold - stopThreshold))
    else:
      driveSpeed = max(0, (proxFR - stopThreshold)/(fearThreshold - stopThreshold))
      turnSpeed = max(-1, -(fearThreshold - proxFR)/(fearThreshold - stopThreshold))
  elif(proxFL < fearThreshold):  # if an object is detected to its left
    driveSpeed = max(0, (proxFL - stopThreshold)/(fearThreshold - stopThreshold))
    turnSpeed = min(1, (fearThreshold - proxFL)/(fearThreshold - stopThreshold))
  elif(proxFR < fearThreshold):  # if an object is detected to its right
    driveSpeed = max(0, (proxFR - stopThreshold)/(fearThreshold - stopThreshold))
    turnSpeed = max(-1, -(fearThreshold - proxFR)/(fearThreshold - stopThreshold))
  else:
    driveSpeed = 1
    turnSpeed = 0

  driveSpeed *= maxSpeed
  turnSpeed *= maxSpeed

  return driveSpeed, turnSpeed

def controllerDrive():
  if(controllerOptions['mapping'] == True):
    controlL = axisValue['xAxisL']
    controlR = (0 - axisValue['yAxisR'])
    driveSpeed = controlR
    turnSpeed = controlL
    turnSpeed /= 4*max(.25, driveSpeed*controllerOptions['powerMult'])
    #^ as turnSpeed*driveSpeed exceeds .25, turning sensitivity is decreased
    #^ this makes the turning stick less sensitive at high speeds
  else:
    controlL = (0 - axisValue['yAxisL'])
    controlR = (0 - axisValue['yAxisR'])
    driveSpeed = (controlL + controlR)/2
    turnSpeed = (controlL - controlR)/2
    
  driveSpeed *= controllerOptions['invertMult']*controllerOptions['powerMult']
  turnSpeed *= controllerOptions['invertMult']*controllerOptions['powerMult']
  turnSpeed = max(-.5, min(.5, turnSpeed))

  return driveSpeed, turnSpeed

def getDriveInstructions(autonomous, proxFL, proxFR, proxBL, proxBR): # turns controller readings into turn/drive commands
  if autonomous:
    driveSpeed, turnSpeed = autonomousDrive(proxFL, proxFR, proxBL, proxBR)
    
  else:
    driveSpeed, turnSpeed = controllerDrive()

  return driveSpeed, turnSpeed
   

#### Serial communication functions ####
def serialWrite(port, portName, LPFcutoff, driveSpeed, turnSpeed, tiltDegrees, swivelDegrees): # writes serial commands to Arduino
  outputString = "{0:.2f}:{1:.2f}:{2:.2f}:{3:.0f}:{4:.0f}\n".format(LPFcutoff, driveSpeed, turnSpeed, tiltDegrees, swivelDegrees)
  try:
    length = len(outputString)
    written = port.write(bytes(outputString, 'UTF-8'))
    if(written != length):
      print("{0} bytes instead of {1} bytes were written.".format(written, length))
  except serial.serialutil.SerialException:
    print()
    print("Serial connection error!")
    port.close()
    port = initializeSerial(portName)
    port.flushInput()
    port.flushOutput()
    #time.sleep(1)
    print("Serial port '%s' re-opened successfully." % portName)
    print()

  return port


#### Interrupts ####
def checkReboot():
  if(buttonValue['triangle'] == 1 and buttonValue['circle'] == 1 and
     buttonValue['cross'] == 1 and buttonValue['square'] == 1):
    print("Power off command received!")
    pressedTime = time.time()
    while(buttonValue['triangle'] == 1 and buttonValue['circle'] == 1 and
          buttonValue['cross'] == 1 and buttonValue['square'] == 1):
      readController()
      if(time.time() > pressedTime + 3):
        print("System will now shut down.")
        os.system("sudo shutdown -h now")
        raise KeyboardInterrupt
    print("System will now reboot.")
    os.system("sudo shutdown -r now")
    raise KeyboardInterrupt
def checkKeyboardInterrupt():
  events = pygame.event.get()   
  for event in events:
    if(event.type == 10 and event.button == 0):
      raise KeyboardInterrupt

def checkInterrupts():
  checkReboot()
  checkKeyboardInterrupt()


######## Control Loop ########
lastTime = time.time()*1000000
try:
  while True:
    dt = int(time.time()*1000000 - lastTime)       
    lastTime = int(time.time()*1000000)
    
    readController()

    toggleController()
    updateLastValues()
    
    theta, proxFL, proxFR, proxBL, proxBR = serialRead(port)
 
    autonomous = controllerOptions['autonomous']
    driveSpeed, turnSpeed = getDriveInstructions(autonomous, proxFL, proxFR, proxBL, proxBR)

    tiltDegrees = 90 - float(theta)
    swivelDegrees = 90 + turnSpeed*90

    port = serialWrite(port, portName, controllerOptions['smoothing'], driveSpeed, turnSpeed, tiltDegrees, swivelDegrees)

    print(dt, theta, proxFL, proxFR, proxBL, proxBR)  # user monitor

    time.sleep(.02)

    checkInterrupts()
except KeyboardInterrupt:
  port.close()
  print("Serial port %s successfully closed." % portName)
  dualshock.quit()
  print("Controller instance successfully terminated.")
  #camera.close()
  print("Camera instance successfully terminated.")
  sixad = subprocess.Popen(["/usr/bin/sixad", "--stop"],
                           universal_newlines=True)
  print("Sixad process successfully terminated.")
  sys.exit("Program successfully terminated.")
