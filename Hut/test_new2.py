# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import patterns
import patternctrl

# Simple test for NeoPixels on Raspberry Pi
import time

import board

import neopixel   #siehe https://learn.adafruit.com/neopixels-on-raspberry-pi/python-usage

import Pyro4
import threading
import socket
import pyaudio
from scipy.fft import fft
from scipy.signal.windows import blackman
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

pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=0.2, auto_write=False, pixel_order=ORDER
)


p = pyaudio.PyAudio()

#Audiokram
stream = p.open(format=sample_format,
    channels=channels,
    rate=fs,
    frames_per_buffer=chunk,
    input=True)

# Pattern
pattern = patterns.FFTPattern(pixels, stream, chunk, num_pixels)

patternctrlinstance = patternctrl.PatternCtrl([pattern])

def run_pyro():
    daemon = Pyro4.Daemon(Pyro4.socketutil.getIpAddress(socket.gethostname()+'.local'))
    ns = Pyro4.locateNS()
    uri=daemon.register(patternctrlinstance)
    ns.register('Hut',uri)
    print("uri=",uri)
    daemon.requestLoop()

pyrothread = threading.Thread(target=run_pyro)
pyrothread.start()

while True:
    pattern.play()