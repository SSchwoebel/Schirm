from parameters import Parameter

class BasePattern:
    def __init__(self):
        self.parameterdescriptions = []
        self.parameters = {}
        self.name = None

    def play(self, pixels):
        raise NotImplementedError("Subclasses must implement the play method.")

class FFTPattern(BasePattern):
    def __init__(self):
        super().__init__()
        self.name = "FFT Pattern"
        self.parameterdescriptions.append(Parameter("Brightness", 0, 100))
        self.parameters["Brightness"] = 50
        self.maximum = 0

    def play(self, pixels):
        # Implement the FFT pattern logic here, using self.parameters["Brightness"] as needed
        
        # Store data in chunks for 3 seconds
        data = stream.read(chunk, False)

        x = numpy.frombuffer(data,dtype=numpy.int16)
        w = blackman(chunk)
        y = fft(x*w)
        real = y.real[1:chunk//2]

        led_values = resize(real,(num_pixels,1))

        localmax=numpy.max(led_values)
        if  localmax > self.maximum:
            self.maximum = localmax
        led_values = led_values.reshape(num_pixels)
        led_values= numpy.absolute(led_values/self.maximum* self.parameters["Brightness"]* 1000)
        led_values= numpy.clip(led_values,0,255)
        led_values= led_values.astype(int)
        for i in range(num_pixels):
            pixels[i]=(led_values[i]//2,led_values[i]//2,led_values[i])
        pixels.show()