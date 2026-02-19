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
    def __init__(self, pixels, num_pixels):
        # self.super.__init__()
        self.pixels = pixels
        self.num_pixels = pixels

        self.parameters = {"wait": {"value": 0.01, "range": [0.001, 0.1]}}

    def set_parameters(self, parameters):
        self.parameters = parameters

    def get_parameters(self):

        return self.parameters

    def play():
        for j in range(255):
            for i in range(self.num_pixels):
                pixel_index = (i * 256 // self.num_pixels) + j
                pixels[i] = wheel(pixel_index & 255)
            pixels.show()
            time.sleep(self.parameters["wait"]["value"])