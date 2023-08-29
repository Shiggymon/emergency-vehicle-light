from machine import UART, Pin, Timer
from pysm import State, StateMachine, Event
import math
import rp2
import time
import random

##############################
# Configuration of Functionality
##############################
# everything in here are the default values.
# if mapped accordingly they can be overwritten anywhere in the code
dfpVolume = 30 # 0 - 30
roofSampleRate = 10 # samplerate in ms, max 50
frontSampleRate = 10 # samplerate in ms, max 50
reactionTimeout = 5000 # timeout for reaction in ms


# Set an integer to take the value from the corresponding dip switch (starting with 0)
# dip switches are read once during initialization
fConfig={
"dfpActive":2, # output sound after money input
"dfpMultiple":3, # play multiple Tracks (loops through mp3 folder)
"dfpRandom":4, # play Tracks in a random Order. dfpMultiple has to be True as well. 
"keepAliveActive":1, # Activate a constant Load to keep a Powerbank alive
"lightAlwaysActive":0, # keep the light active (only add sound on money input if activated) - not implemented yet
}

###############################
# Configration of blinkPatterns
###############################
# This can be either a one dimensional array applied to all outputs or a two dimensional Array with same length as the pin Array.
# if neither criteria is met the first Array will be used for all outputs
# The definition has to be an Array of times (ms) alternating between on and off (first value is on, can be 0).
roofBlinkPatterns = [150, 110, 60, 120, 60, 330]
frontBlinkPatterns = [120, 60, 50, 100, 50, 200]
# A static Offset between each output can be defined. This is intended to be used with a single pattern for all Outputs.
# Negative values are considered as 0. 
roofOffset = 100
frontOffset = 0

###############################
# Connection of the Peripherals
###############################
dfpTxPinId = 12 # Pin ID
dfpRxPinId = 13 # Pin ID
dfpUartId = 0 # Pin ID
dfpIO1PinId = 28 # Pin ID
roofLedPinIds = [19, 18, 17, 16] # list of Pin IDs
frontLedPinIds = [20] # list of Pin IDs
keepAlivePinId = 15 # Pin ID
dipPinIds = [2, 3, 4, 5, 6, 7] # list of Pin IDs 
moneyLedPinId = 21 # Pin ID
moneySensorPinId = 22 # Pin ID


###############################
# Advanced i/o configuration
###############################
dfpBaudrate = 9600
dfpBits = 8
dfpParity = None
dfpStop = 1
dfpUseIONext = False
# invert can be either True / False for all in-/outputs or an Array with same length as the pin Array.
# if neither criteria is met the pins will not be inverted (= False)
invertRoofLeds = False
invertFrontLeds = False
invertKeepAlive = False
invertDips = True # can only be scalar, despite pin ids being a list
invertMoneyLed = False
invertMoneySensor = True
invertDfpIO1 = True
# should pullups/-downs be used for the inputs?
pullDip = Pin.PULL_UP
pullMoneySensor = None

###############################
# initialization of HW Peripherals
###############################
# Uart for DFP
dfpTxPin = Pin(dfpTxPinId, Pin.ALT_UART)
dfpRxPin = Pin(dfpRxPinId, Pin.IN)
dfpUart = UART(dfpUartId, dfpBaudrate)
dfpUart.init(dfpBaudrate, bits=dfpBits, parity=dfpParity, stop=dfpStop, tx=dfpTxPin, rx=dfpRxPin)
dfpIO1Pin = Pin(dfpIO1PinId, Pin.OUT)
# Pins for LEDs
roofLedPins = []
for p in roofLedPinIds:
    roofLedPins.append(Pin(p, Pin.OUT))
frontLedPins = []
for p in frontLedPinIds:
    frontLedPins.append(Pin(p, Pin.OUT))
# Additional Pins
keepAlivePin = Pin(keepAlivePinId, Pin.OUT)
dipPins = []
for p in dipPinIds:
    dipPins.append(Pin(p, mode=Pin.IN, pull=pullDip))
moneyLedPin = Pin(moneyLedPinId, Pin.OUT)
moneysensorPin = Pin(moneySensorPinId, mode=Pin.IN, pull=pullMoneySensor)

hwTimer = Timer()

##############################################
# functions for the communication with the DFPlayer
##############################################
def DFPcalcChecksum(packet):
    sum = 0
    for i in range(1, 7):
        sum += packet[i]
    return -sum

def DFPsetChecksum(packet):
    sum = DFPcalcChecksum(packet)
    packet[7] = sum >> 8
    packet[8] = sum & 0xff

def DFPSendCommand(command, param1=0, param2=0):
    sendBytes = bytearray(10)
    sendBytes[0] = 0x7E 	# starting Byte
    sendBytes[1] = 0xFF		# Command Version
    sendBytes[2] = 0x6		# Command Length
    sendBytes[3] = command	# Command Byte
    sendBytes[4] = 0x0		# ACK? 
    sendBytes[5] = param1	# High Param Byte
    sendBytes[6] = param2	# Low Param Byte
    sendBytes[7] = 0		# High Checksum Byte
    sendBytes[8] = 0		# Low Checksum Byte
    sendBytes[9] = 0xEF		# End Byte
    DFPsetChecksum(sendBytes)
    dfpUart.write(sendBytes)

def DFPSendQuery(command, param1=0, param2=0):
    retry=True
    while(retry):
        DFPSendCommand(command, param1, param2)
        dfpUart.flush()
        time.sleep(0.2)
        recv = dfpUart.read()
        if not recv: # timeout
            return 
        if len(recv) == 10 and recv[1] == 0xFF and recv[9] == 0xEF:
            retry = False
    return recv

def DFPPlaySelected(param1=0, param2=0):
    DFPSendCommand(0x03, param1, param2)

def DFPNext():
    if dfpUseIONext:
        setOutput(dfpIO1Pin, False, invertDfpIO1)
        time.sleep(0.08)
        setOutput(dfpIO1Pin, True, invertDfpIO1)
    else:
        DFPSendCommand(0x01)

def DFPPrevious():
    DFPSendCommand(0x02)

def DFPSetVolume(volume):
    DFPSendCommand(0x06, 0, volume)

def DFPResetModule():
    DFPSendCommand(0x0C)

def DFPPlay():
    DFPSendCommand(0x0D)

def DFPPause():
    DFPSendCommand(0x0E)

def DFPGetFileCount():
    resp = DFPSendQuery(0x48)
    fileCount = None
    if resp:
        if resp[3] == 0x48:
            fileCount = (resp[5] << 8) + resp[6]
    return fileCount

##############################
# Definition of helper functions
##############################
# output msg as hex values
def printHex(msg):
    for h in msg:
        print(hex(h), end=" ")

# initialize the counter variables for blink patterns
def initCounters(leds, offset, samplerate):
    offset = max(0, offset) # limit to positive offsets
    off = 0
    counters = []
    for c in range(len(leds)):
        counters.append(-off)
        off += round(offset/samplerate)
    return counters

# initialize the blink sequence
def initBlinkSequence(leds, blinkPatterns, samplerate):
    blinkSequences = []
    if len(blinkPatterns) == len(leds) and type(blinkPatterns[0]) == list:
        # assume one list for each led
        pass
    elif type(blinkPatterns[0]) == list:
        # use first list for all leds
        pattern = blinkPatterns[0]
        blinkPatterns = [pattern] * len(leds)
    elif len(blinkPatterns) > 0:
        # assume integer list used for all leds
        pattern = blinkPatterns
        blinkPatterns = [pattern] * len(leds)
    # iterate over the patterns
    for i in range(len(leds)):
        sequence = []
        for idj,j in enumerate(blinkPatterns[i]):
            for k in range(math.floor(j/samplerate)):
                sequence.append((idj+1)%2)
        blinkSequences.append(sequence)
    return blinkSequences

# start the blinking
def startBlink():
    global roofCounter
    global frontCounter
    global blinkActive
    # check if blink is active, if yes do nothing
    if not blinkActive:        
        # initialize Counters
        roofCounter = initCounters(roofLedPins, roofOffset, roofSampleRate)
        frontCounter = initCounters(frontLedPins, frontOffset, frontSampleRate)
        # start the pio statemachine
        roofSmPio.active(1)
        frontSmPio.active(1)
    blinkActive = True

# stop the blinking
def stopBlink():
    global blinkActive
    blinkActive = False
    # stop the pio statemachine
    roofSmPio.active(0)
    frontSmPio.active(0)
    while roofSmPio.active() or frontSmPio.active():
        pass
    setOutput(roofLedPins, 0, invertRoofLeds)
    setOutput(frontLedPins, 0, invertFrontLeds)

# start the Playback
def startSound():
    global lastPlayed
    if time.ticks_ms() - lastPlayed > 1000:
        if fConfig["dfpMultiple"]:
            if fConfig["dfpRandom"] and dfpFileCount != None:
                songNumber = random.randint(1, dfpFileCount)
                DFPPlaySelected(songNumber >> 8, songNumber & 0xFF)
            else:
                DFPNext()
        else:
            DFPPlaySelected(0,0)
    lastPlayed = time.ticks_ms()

# stop the Playback
def stopSound():
    DFPPause()

# update the given config (read dip switches)
# This is designed as a single shot function, since it removes all integers in the config
def updateConfig(config):
    for i in config:
        if type(config[i]) == int:
            # read value from corresponding dip switch
            config[i] = bool(getInput(dipPins[config[i]], invertDips))
# set output and apply inversion if necessary
# can accept a list of leds as well
def setOutput(outputs, value, invert=False):
    if type(outputs) != list:
        outputs = [outputs]
    if type(value) != list:
        value = [value]
    if type(invert) != list:
        invert = [invert]
    if len(value) != len(outputs):
        value = [value[0]] * len(outputs)
    if len(invert) != len(outputs):
        invert = [invert[0]] * len(outputs)
    for iOutp,outp in enumerate(outputs):
        outp.value(value[iOutp]^invert[iOutp])

# get input and apply inversion if necessary
def getInput(inp, invert=False):
    return inp.value() ^ invert

# bootanimation loading all lights
def startBootAnimation(leds, invert):
    if type(invert) != list:
        invert = [invert]
    if len(invert) != len(leds):
        invert = [invert[0]] * len(leds)
    for iLed,led in enumerate(leds):
        setOutput(led, True, invert[iLed])
        time.sleep(0.1)

# bootanimation blinking all lights synchronously
def blinkBootAnimation(leds, invert):
    for i in range(3):
        setOutput(leds, False, invert)
        time.sleep(0.1)
        setOutput(leds, True, invert)
        time.sleep(0.1)        

# bootanimation unloading all lights
def stopBootAnimation(leds, invert):
    if type(invert) != list:
        invert = [invert]
    if len(invert) != len(leds):
        invert = [invert[0]] * len(leds)
    for iLed,led in enumerate(leds):
        setOutput(led, False, invert[iLed])
        time.sleep(0.1)

###############################
# Define the Statemachine State Event handlers
###############################
# Leaving the init State
# Put everything in here that should be done once at startup of the System
def onExitInit(state, event):
    global roofBlinkSequences
    global frontBlinkSequences
    global dfpFileCount
    # start bootanimantion
    startBootAnimation(roofLedPins+frontLedPins, invertRoofLeds and invertFrontLeds)
    # read settings from dip switches
    updateConfig(fConfig)
    # init the blinksequences
    roofBlinkSequences = initBlinkSequence(roofLedPins, roofBlinkPatterns, roofSampleRate)
    frontBlinkSequences = initBlinkSequence(frontLedPins, frontBlinkPatterns, frontSampleRate)
    # activate the money input Led
    setOutput(moneyLedPin, True, invertMoneyLed)
    # Reset DFP Module and set Volume
    setOutput(dfpIO1Pin, True, invertDfpIO1)
    DFPResetModule()
    time.sleep(2)
    DFPSetVolume(dfpVolume)
    time.sleep(1)
    dfpFileCount = DFPGetFileCount()
    # set keepalive according to settings
    if fConfig["keepAliveActive"]:
        setOutput(keepAlivePin, True, invertKeepAlive)
    # connect money sensor input to interrupt handler
    moneySensorTrigger = None
    if invertMoneySensor:
        moneySensorTrigger = Pin.IRQ_RISING
    else:
        moneySensorTrigger = Pin.IRQ_FALLING
    moneysensorPin.irq(handler=lightInterrupt, trigger=moneySensorTrigger)
    blinkBootAnimation(roofLedPins+frontLedPins, invertRoofLeds and invertFrontLeds)
    stopBootAnimation(roofLedPins+frontLedPins, invertRoofLeds and invertFrontLeds)
    
# Entering the active State
# Put everything in here that should be done after money has been thrown into
# This will be executed on every input
def onEnterActive(state, event):
    # start the sound playback
    if fConfig["dfpActive"]:
        startSound()
    # start the blinking
    startBlink()    
    # start the timer for the timeout
    hwTimer.init(period=reactionTimeout, mode=Timer.ONE_SHOT, callback=lambda t:sm.dispatch(Event("timeout")))

# Leaving the active State
# Put everything in here, that should be done when stopping the action
# need to check if this will be run on repeated input
def onExitActive(state, event):
    pass

# Entering the ready State
# Put everything in here that needs to be done to get into the initial system State.
def onEnterReady(state, event):
    # stop the playback if not stopped already
    stopSound()
    # set blink state according to the settings
    if fConfig["lightAlwaysActive"]:
        startBlink() # activate if always active
    else:
        stopBlink() #stop otherwise
    
###############################
# Set up the Statemachine
###############################
# Define the states
initState = State("init")
readyState = State("ready")
activeState = State("active")

# create the Statemachine and add States
sm = StateMachine("blink")
sm.add_state(initState, initial=True)
sm.add_state(readyState)
sm.add_state(activeState)

# define the SateMachine transitions
sm.add_transition(initState, readyState, events=["ready"])
sm.add_transition(readyState, activeState, events=["money"])
sm.add_transition(activeState, readyState, events=["timeout"])
sm.add_transition(activeState, activeState, events=["money"])

# connect the state handlers to the states
initState.handlers = {"exit": onExitInit}
readyState.handlers = {"enter": onEnterReady}
activeState.handlers = {"enter": onEnterActive, "exit": onExitActive}

# Initialize the statemachine
sm.initialize()

###############################
# Define the pio interrupt handlers
###############################
def roofInterrupt(callback):
    # determine the current led states
    values = []
    for idSeq, seq in enumerate(roofBlinkSequences):
        if roofCounter[idSeq] >= 0:
            values.append(seq[roofCounter[idSeq]%len(seq)])
        else:
            values.append(0)
    # set the outputs
    if roofSmPio.active():
        setOutput(roofLedPins, values, invertRoofLeds)
    # advance the counter
    roofCounter[:] = [x+1 for x in roofCounter]

def frontInterrupt(callback):
    # determine the current led states
    values = []
    for idSeq, seq in enumerate(frontBlinkSequences):
        if frontCounter[idSeq] >= 0:
            values.append(seq[frontCounter[idSeq]%len(seq)])
        else:
            values.append(0)
    # set the outputs
    if frontSmPio.active():
        setOutput(frontLedPins, values, invertFrontLeds)
    # advance the counter
    frontCounter[:] = [x+1 for x in frontCounter]

###############################
# Setup the pio
###############################
# define the pio interrupt functions
# the functions throw a nonblocking interrupt every 100 steps.
# Frequency can be adjusted by changing the step frequency in the Statemachine initialization
@rp2.asm_pio()
def timer0_100():
    # Cycles: 1 + 6 + 3 * (30 + 1) = 100
    irq(0)
    set(x, 2)	[5]
    label("loop")
    nop()		[29]
    jmp(x_dec, "loop")

# repeat function for second interrupt
@rp2.asm_pio()
def timer1_100():
    # Cycles: 1 + 6 + 3 * (30 + 1) = 100
    irq(1)
    set(x, 2)	[5]
    label("loop")
    nop()		[29]
    jmp(x_dec, "loop")

# initialize pio statemachines
roofSmPio = rp2.StateMachine(0, timer0_100, freq=10000) # interrupt timer for roof Leds
frontSmPio = rp2.StateMachine(1, timer1_100, freq=10000) # interrupt timer for front Leds
# attach the interrupt handlers to the pio statemachines
roofSmPio.irq(roofInterrupt)
frontSmPio.irq(frontInterrupt)

###############################
# Setup the input interrupt function
###############################
# interrupt function for external input
def lightInterrupt(callback):
    sm.dispatch(Event("money"))

###############################
# Variables for Functionality
###############################
roofCounter = []
frontCounter = []

roofBlinkSequences = []
frontBlinkSequences = []

blinkActive = False
lastPlayed = 0
dfpFileCount = None

# Switch Statemachine to ready State doing the startup process
sm.dispatch(Event("ready"))
