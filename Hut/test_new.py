# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Simple test for NeoPixels on Raspberry Pi
import time

import board

import neopixel

import Pyro4
import threading
import socket
import pyaudio
from scipy.fft import fft
import numpy
from skimage.transform import resize
# Choose an open pin connected to the Data In of the NeoPixel strip, i.e. board.D18
# NeoPixels must be connected to D10, D12, D18 or D21 to work.
pixel_pin = board.D18

# The number of NeoPixels
num_pixels = 43

# The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
ORDER = neopixel.RGB

#Recording Parameters
#chunk = 1024  # Record in chunks of 1024 samples
sample_format = pyaudio.paInt16  # 16 bits per sample
channels = 1
fs = 48000  # Record at 44100 samples per second
seconds = 0.01
chunk = int(fs*seconds)

#FFT Parameters
fft_factor=0.00000001
pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=0.2, auto_write=False, pixel_order=ORDER
)

#Patternnormalization
maximum = 1.0

p = pyaudio.PyAudio()

#Audiokram
stream = p.open(format=sample_format,
    channels=channels,
    rate=fs,
    frames_per_buffer=chunk,
    input=True)

def wheel(pos):
    # Input a value 0 to 255 to get a color value.
    # The colours are a transition r - g - b - back to r.
    if pos < 0 or pos > 255:
        r = g = b = 0
    elif pos < 85:
        r = int(pos * 3)
        g = int(255 - pos * 3)
        b = 0
    elif pos < 170:
        pos -= 85
        r = int(255 - pos * 3)
        g = 0
        b = int(pos * 3)
    else:
        pos -= 170
        r = 0
        g = int(pos * 3)
        b = int(255 - pos * 3)
    return (r, g, b) if ORDER in {neopixel.RGB, neopixel.GRB} else (r, g, b, 0)


def rainbow_cycle(wait):
    for j in range(255):
        for i in range(num_pixels):
            pixel_index = (i * 256 // num_pixels) + j
            pixels[i] = wheel(pixel_index & 255)
        pixels.show()
        time.sleep(wait)

def fft_pattern():
    global maximum
    # Store data in chunks for 3 seconds
    data = stream.read(chunk)

    x = numpy.frombuffer(data,dtype=numpy.int16)
    y = fft(x)
    real = y.real

    led_values = resize(real,(num_pixels,1))

    localmax=numpy.max(led_values)
    if  localmax > maximum:
        maximum = localmax
    print(maximum)
    led_values = led_values.reshape(num_pixels)
    led_values= abs(led_values/maximum* 255)
    led_values= led_values.astype(int)
    for i in range(num_pixels):
        pixels[i]=(0,0,led_values[i])
    pixels.show()

@Pyro4.expose
class Pattern(object):
    def __init__(self):
        self.PatternNr=4

    @property
    def Nr(self):             # exposed as 'proxy.attr' remote attribute
        return self.PatternNr

    @Nr.setter
    def Nr(self, value):      # exposed as 'proxy.attr' writable
        self.PatternNr = value

patterninstance = Pattern()

def run_pyro():
    daemon = Pyro4.Daemon(Pyro4.socketutil.getIpAddress(socket.gethostname()+'.local'))
    ns = Pyro4.locateNS()
    uri=daemon.register(patterninstance)
    ns.register('Hut',uri)
    print("uri=",uri)
    daemon.requestLoop()

pyrothread = threading.Thread(target=run_pyro)
pyrothread.start()

while True:
    if patterninstance.Nr==0:
        pixels.fill((255,0,0))
        pixels.show()
    elif patterninstance.Nr==1:
        pixels.fill((0,255,0))
        pixels.show()
    elif patterninstance.Nr==2:
        pixels.fill((0,0,255))
        pixels.show()
    elif patterninstance.Nr==3:
        rainbow_cycle(0.001)
    elif patterninstance.Nr==4:
        fft_pattern()
    else:
        # Comment this line out if you have RGBW/GRBW NeoPixels
        pixels.fill((255, 0, 0))
        # Uncomment this line if you have RGBW/GRBW NeoPixels
        # pixels.fill((255, 0, 0, 0))
        pixels.show()
        time.sleep(1)

        # Comment this line out if you have RGBW/GRBW NeoPixels
        pixels.fill((0, 255, 0))
        # Uncomment this line if you have RGBW/GRBW NeoPixels
        # pixels.fill((0, 255, 0, 0))
        pixels.show()
        time.sleep(1)

        # Comment this line out if you have RGBW/GRBW NeoPixels
        pixels.fill((0, 0, 255))
        # Uncomment this line if you have RGBW/GRBW NeoPixels
        # pixels.fill((0, 0, 255, 0))
        pixels.show()
        time.sleep(1)

        rainbow_cycle(0.001)  # rainbow cycle with 1ms delay per step

