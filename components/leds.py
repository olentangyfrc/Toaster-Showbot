import math
import random
from wpilib import AddressableLED, Color8Bit, Timer, SmartDashboard
import enum

class LEDMode(enum.IntEnum):
    OFF = 0
    WORK_LIGHT = 1
    METEOR = 2
    PULSE = 3
    RAINBOW = 4
    SCROLL = 5
    BREATH = 6
    FLAMES = 7
    POLICE = 8
    SNAKE = 9
    MATRIX = 10
    BOUNCE = 11
    CONFETTI = 12

class LEDController:
    led_strip: AddressableLED

    def __init__(self):
        self.total_len = 29
        self.led_data = [AddressableLED.LEDData() for _ in range(self.total_len)]
        
        self.mode = LEDMode.OFF
        self.color = Color8Bit(0, 255, 0)
        self.speed_bpm = 30.0
        self.tail_length = 5
        
        self.timer = Timer()
        self.hue = 0
        self.meteor_pos = 0.0
        self.meteor_dir = 1
        self.heat = [0] * self.total_len

    def setup(self):
        self.led_strip.setLength(self.total_len)
        self.led_strip.start()
        self.timer.start()

    def cycle_mode(self, direction: int):
        modes = list(LEDMode)
        current_index = modes.index(self.mode)
        new_index = (current_index + direction) % len(modes)
        self.mode = modes[new_index]

    def _fade_all(self):
        """Standard fading for trails - modified to be more aggressive for shorter tails."""
        # Using a higher divisor prevents the 'spacing' by keeping the trail dense
        fade_amount = int(255 / (self.tail_length + 1))
        for led in self.led_data:
            led.setRGB(
                max(0, led.r - fade_amount), 
                max(0, led.g - fade_amount), 
                max(0, led.b - fade_amount)
            )

    def execute(self):
        SmartDashboard.putString("LED/Mode", self.mode.name)

        if self.mode == LEDMode.OFF:
            for led in self.led_data: led.setRGB(0, 0, 0)

        elif self.mode == LEDMode.WORK_LIGHT:
            for i in range(self.total_len): self.led_data[i].setRGB(255, 245, 190)

        elif self.mode == LEDMode.METEOR:
            self._fade_all()
            # Dense movement logic: Step size is small enough to hit every LED
            step = (self.speed_bpm / 60.0) * 1.5
            self.meteor_pos += (self.meteor_dir * step)
            
            if self.meteor_pos >= self.total_len - 1:
                self.meteor_pos = self.total_len - 1
                self.meteor_dir = -1
            elif self.meteor_pos <= 0:
                self.meteor_pos = 0
                self.meteor_dir = 1
            
            # Draw the lead pixel
            self.led_data[int(self.meteor_pos)].setRGB(self.color.red, self.color.green, self.color.blue)

        elif self.mode == LEDMode.PULSE:
            # Heartbeat logic lives here!
            time = self.timer.get() * (self.speed_bpm / 60.0)
            cycle = time % 2.0
            lub = math.exp(-pow(cycle - 0.3, 2) / 0.01)
            dub = 0.6 * math.exp(-pow(cycle - 0.6, 2) / 0.01)
            val = max(lub, dub)
            
            for led in self.led_data:
                led.setRGB(
                    int(self.color.red * val), 
                    int(self.color.green * val), 
                    int(self.color.blue * val)
                )

        elif self.mode == LEDMode.RAINBOW:
            self.hue += (self.speed_bpm / 60.0)
            for i in range(self.total_len):
                h = int(self.hue + (i * 180 / self.total_len)) % 180
                self.led_data[i].setHSV(h, 255, 255)

        elif self.mode == LEDMode.SCROLL:
            offset = int(self.timer.get() * (self.speed_bpm / 2))
            for i in range(self.total_len):
                if (i + offset) % 4 == 0:
                    self.led_data[i].setRGB(self.color.red, self.color.green, self.color.blue)
                else:
                    self.led_data[i].setRGB(0, 0, 0)

        elif self.mode == LEDMode.BREATH:
            val = abs(math.sin(self.timer.get() * (self.speed_bpm / 60.0) * math.pi))
            for led in self.led_data:
                led.setRGB(int(self.color.red * val), int(self.color.green * val), int(self.color.blue * val))
        elif self.mode == LEDMode.FLAMES:
            # 1. GENTLE COOLDOWN (keeping your slow-burn speed)
            for i in range(self.total_len):
                self.heat[i] = max(0, self.heat[i] - random.randint(0, 4))

            # 2. STRONGER DIFFUSION
            for k in range(self.total_len - 1, 2, -1):
                self.heat[k] = (self.heat[k - 1] + self.heat[k - 2] + self.heat[k - 2]) // 3

            # 3. SPARKING
            if random.random() < 0.5: 
                spark_pixel = random.randint(0, 2)
                self.heat[spark_pixel] = min(255, self.heat[spark_pixel] + random.randint(180, 255))

            # 4. SECTIONAL DIP (for the top 10 LEDs)
            for i in range(19, self.total_len):
                if random.random() < 0.15:
                    self.heat[i] = max(0, self.heat[i] - random.randint(0, 15))

            # 5. FIXED COLOR MAPPING (No more "too much white")
            for i in range(self.total_len):
                h = self.heat[i]
                
                # RED: Always matches the heat level
                r = h 
                
                # GREEN: We increase the subtraction to 130 (was 80)
                # This keeps the fire RED/ORANGE longer before turning YELLOW
                g = max(0, h - 130) 
                
                # BLUE: We increase the subtraction to 245 (was 200)
                # This ensures WHITE only appears at the absolute peak heat
                b = max(0, h - 245) 
                
                self.led_data[i].setRGB(r, g, b)

        elif self.mode == LEDMode.SNAKE:
            self._fade_all()
            speed = self.timer.get() * (self.speed_bpm / 4)
            pos = int(speed) % self.total_len
            self.led_data[pos].setRGB(self.color.red, self.color.green, self.color.blue)

        elif self.mode == LEDMode.MATRIX:
            self._fade_all()
            if random.random() < (self.speed_bpm / 100):
                self.led_data[random.randint(0, self.total_len-1)].setRGB(0, 255, 0)

        elif self.mode == LEDMode.BOUNCE:
            self._fade_all()
            pos = (math.sin(self.timer.get() * (self.speed_bpm / 10)) + 1) / 2 * (self.total_len - 1)
            self.led_data[int(pos)].setRGB(self.color.red, self.color.green, self.color.blue)

        elif self.mode == LEDMode.CONFETTI:
            self._fade_all()
            if random.random() < (self.speed_bpm / 100):
                i = random.randint(0, self.total_len-1)
                temp = AddressableLED.LEDData()
                temp.setHSV(random.randint(0, 180), 255, 255)
                self.led_data[i].setRGB(temp.r, temp.g, temp.b)

        # Update hardware
        self.led_strip.setData(self.led_data)

        # Output for Multi Color View in Elastic
        hex_data = [f"#{int(led.r):02x}{int(led.g):02x}{int(led.b):02x}" for led in self.led_data]
        SmartDashboard.putStringArray("LEDs/Output", hex_data)