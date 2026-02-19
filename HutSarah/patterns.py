import neopixel
import time

class BasePattern(object):
    def __init__(self):
        self.pixels = pixels
        self.num_pixels = num_pixels

    def set_parameters(self, **kwargs):
        pass

    def get_parameters(self, **kwargs):
        pass

    def play(self, **kwargs):
        pass


class RainbowCycle(BasePattern):
    def __init__(self, pixels, num_pixels, order):
        # self.super.__init__()
        self.pixels = pixels
        self.num_pixels = num_pixels
        self.order = order

        self.parameters = {"wait": {"value": 0.01, "range": [0.001, 0.1]}}

    def set_parameters(self, parameters):
        self.parameters = parameters

    def get_parameters(self):

        return self.parameters

    def play(self):
        for j in range(255):
            for i in range(self.num_pixels):
                pixel_index = (i * 256 // self.num_pixels) + j
                self.pixels[i] = self._wheel(pixel_index & 255)
            self.pixels.show()
            time.sleep(self.parameters["wait"]["value"])

    def _wheel(self, pos):
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
        return (r, g, b) if self.order in {neopixel.RGB, neopixel.GRB} else (r, g, b, 0)
