from parameters import Parameter
import numpy
from scipy.fftpack import fft
from scipy.signal.windows import blackman
from skimage.transform import resize
import pyaudio
import neopixel
import board

class BasePattern:
    def __init__(self):
        self.parameterdescriptions = []
        self.parameters = {}
        self.name = None

    def play(self):
        raise NotImplementedError("Subclasses must implement the play method.")

class FFTPattern(BasePattern):
    def __init__(self, pixels, stream, chunk, num_pixels):
        super().__init__()
        self.name = "FFT Pattern"
        self.parameterdescriptions.append(Parameter("Brightness", 0, 100))
        self.parameters["Brightness"] = 50

        self.pixels = pixels
        self.stream = stream
        self.chunk = chunk
        self.num_pixels = num_pixels    

        self.maximum = 0


    def play(self):
        # Implement the FFT pattern logic here, using self.parameters["Brightness"] as needed
        
        # Store data in chunks for 3 seconds
        data = self.stream.read(self.chunk, False)

        x = numpy.frombuffer(data,dtype=numpy.int16)
        w = blackman(self.chunk)
        y = fft(x*w)
        real = y.real[1:self.chunk//2]

        led_values = resize(real,(self.num_pixels,1))

        localmax=numpy.max(led_values)
        if  localmax > self.maximum:
            self.maximum = localmax
        led_values = led_values.reshape(self.num_pixels)
        led_values= numpy.absolute(led_values/self.maximum* self.parameters["Brightness"]* 1000)
        led_values= numpy.clip(led_values,0,255)
        led_values= led_values.astype(int)
        for i in range(self.num_pixels):
            self.pixels[i]=(led_values[i]//2,led_values[i]//2,led_values[i])
        self.pixels.show()