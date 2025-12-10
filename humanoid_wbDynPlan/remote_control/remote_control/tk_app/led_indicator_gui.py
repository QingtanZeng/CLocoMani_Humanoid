/* Computational Legged Robots: Planning and Control  */

import tkinter as tk


class LEDIndicatorGui(tk.Frame):
    def __init__(
        self, parent, label_text="LED", size=20, color_on="#4a90e2", color_off="#363636"
    ):
        super().__init__(parent)
        self.configure(bg="#2c2c2c")  # Match dark background
        self.color_on = color_on
        self.color_off = color_off

        # Create Label with matching style
        self.label = tk.Label(
            self,
            text=label_text,
            bg="#2c2c2c",
            fg="#ffffff",
            font=("Helvetica", 10),  # Match other controls' font
        )
        self.label.pack(side=tk.LEFT, padx=5)

        # Create Canvas for LED
        self.canvas = tk.Canvas(
            self, width=size, height=size, bg="#2c2c2c", highlightthickness=0
        )
        self.canvas.pack(side=tk.LEFT)

        # Create the LED circle with modern styling
        padding = size * 0.1
        self.led = self.canvas.create_oval(
            padding,
            padding,
            size - padding,
            size - padding,
            fill=self.color_off,
            outline="#4a90e2",  # Match button border color
            width=1,
        )

    def set_state(self, state):
        """Set LED state: True for on, False for off"""
        if state:
            # When on, use the modern blue color with a light border
            self.canvas.itemconfig(
                self.led, fill=self.color_on, outline="#5ca0f2"
            )  # Lighter blue for depth
        else:
            # When off, use the darker gray with standard border
            self.canvas.itemconfig(self.led, fill=self.color_off, outline="#4a90e2")
