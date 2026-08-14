import time

class FswState:
    '''Update the state of the mission using ONLY barometric data'''

    def __init__(self, launch_altitude):
        self.current_state = "LAUNCH_PAD"
        self.launch_altitude = launch_altitude
        self.maximum_height = launch_altitude
        self.previous_altitude = launch_altitude
        
        # Counters for debouncing state transitions
        self.counters = {
            "ascent": 0,
            "apogee": 0,
            "descent": 0,
            "probe_release": 0,
            "payload_release": 0,
            "landed": 0
        }
        self.is_active = True

    def update(self, pressure, altitude):
        '''Update the state of the mission based on purely barometric readings'''
        
        # 1. Continuously track the maximum height
        if altitude > self.maximum_height:
            self.maximum_height = altitude

        # 2. State Machine Logic
        if self.current_state == "LAUNCH_PAD":
            # ASCENT: Altitude is reliably 5 meters above launch site
            if altitude > (self.launch_altitude + 5.0):
                self.counters["ascent"] += 1
                if self.counters["ascent"] > 4:
                    self.current_state = "ASCENT"
            else:
                self.counters["ascent"] = 0

        elif self.current_state == "ASCENT":
            # APOGEE: We have dropped 3 meters below our highest recorded point
            if altitude < (self.maximum_height - 3.0):
                self.counters["apogee"] += 1
                if self.counters["apogee"] > 4:
                    self.current_state = "APOGEE"
            else:
                self.counters["apogee"] = 0

        elif self.current_state == "APOGEE":
            # DESCENT: We have dropped 10 meters below apogee, definitively falling
            if altitude < (self.maximum_height - 10.0):
                self.counters["descent"] += 1
                if self.counters["descent"] > 4:
                    self.current_state = "DESCENT"
            else:
                self.counters["descent"] = 0

        elif self.current_state == "DESCENT":
            # PROBE RELEASE: Reached 80% of the maximum recorded height
            if altitude <= 0.8 * self.maximum_height:
                self.counters["probe_release"] += 1
                if self.counters["probe_release"] > 4:
                    self.current_state = "PROBE_RELEASE"
            else:
                self.counters["probe_release"] = 0

        elif self.current_state == "PROBE_RELEASE":
            # PAYLOAD RELEASE: Approaching the ground (e.g., 15 meters above launch pad)
            if altitude <= (self.launch_altitude + 15.0):
                self.counters["payload_release"] += 1
                if self.counters["payload_release"] > 4:
                    self.current_state = "PAYLOAD_RELEASE"
            else:
                self.counters["payload_release"] = 0

        elif self.current_state == "PAYLOAD_RELEASE":
            # LANDED: We are near the ground AND altitude is no longer changing
            altitude_fluctuation = abs(altitude - self.previous_altitude)
            
            # If altitude changes by less than 0.5m between readings and we are low
            if altitude_fluctuation < 0.5 and altitude <= (self.launch_altitude + 25.0):
                self.counters["landed"] += 1
                if self.counters["landed"] > 15:  # Require a longer time (1.5s) of no movement
                    self.current_state = "LANDED"
                    self.is_active = False
            else:
                self.counters["landed"] = 0
                
        # Update previous altitude for the next loop's calculation
        self.previous_altitude = altitude


# --- Main Flight Loop ---

def read_barometer():
    '''Mock function to read the BMP280/BME280 sensor'''
    # Return mock data: pressure, altitude
    return 1010.0, 150.0 

# Baseline calibration (averaging 10-20 readings before flight)
launch_pad_altitude_calibration = 145.0 

fsw = FswState(launch_altitude=launch_pad_altitude_calibration)

# FSW Loop running at a fixed 10Hz
while fsw.is_active:
    
    # 1. Read sensor
    curr_pressure, curr_alt = read_barometer()
    
    # 2. Update State Machine
    fsw.update(curr_pressure, curr_alt)
    
    # 3. Telemetry / Logging
    print(f"State: {fsw.current_state} | Alt: {curr_alt:.2f}m")
    
    # 4. Enforce sample rate (10 Hz = 0.1 seconds)
    time.sleep(0.1) 

print("Mission Complete. CanSat Landed.")
