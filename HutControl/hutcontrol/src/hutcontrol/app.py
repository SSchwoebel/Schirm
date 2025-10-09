"""
Controls the hat
"""

import toga
from toga.style.pack import COLUMN, ROW
import Pyro4


class HutControl(toga.App):
    def startup(self):
        """Construct and show the Toga application.

        Usually, you would add your application to a main content box.
        We then create a main window (with a name matching the app), and
        show the main window.
        """
        self.hut=Pyro4.Proxy("PYRONAME:Hut@hut.local:9090")

        main_box = toga.Box(direction=COLUMN)

        nr_label = toga.Label(
            "Pattern Number: ",
            margin=(0, 5),
        )
        self.nr_input = toga.TextInput(flex=1)
        self.nr_input.value = str(self.hut.Nr)

        nr_box = toga.Box(direction=ROW, margin=5)
        nr_box.add(nr_label)
        nr_box.add(self.nr_input)

        button = toga.Button(
            "Set Parameters",
            on_press=self.set_parameters,
            margin=5,
        )

        main_box.add(nr_box)
        main_box.add(button)

        self.main_window = toga.MainWindow(title=self.formal_name)
        self.main_window.content = main_box
        self.main_window.show()

    def set_parameters(self, widget):
        self.hut.Nr  = int(self.nr_input.value)

def main():
    return HutControl()
