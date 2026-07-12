'''FSW Logic Module'''
pressure = None  # value from barometric sensor
altitude = None  # value from the International Standard Barometric Formula
launch_site_pressure = 1013.25  # value from barometric sensor at launch site
vertical_velocity = None  # value from accelerometer sensor
maximum_height = None  # maximum height reached during ascent
mode = None  # mode of the mission, can be CX_ON or CX_OFF


class FswState:
    '''Update the state of the mission'''

    def __init__(self):
        self.current_state = "LAUNCH_PAD"
        self.zero_calibration = 0  # calibration of the cansat for the sero altitude
        self.maximum_height = 0  # maximum height reached during ascent
        self.ascent_counter = 0  # counter for the number of readings before state transition to ascent
        self.apogee_counter = 0  # counter for the number of readings before state transition to apogee
        # counter for the number of readings before state transition to descent
        self.descent_counter = 0
        # counter for the number of readings before state transition to probe release
        self.probe_release_counter = 0
        # counter for the number of readings before state transition to payload release
        self.payload_release_counter = 0
        self.landed_counter = 0  # counter for the number of readings before state transition to landed
        global mode
        mode = "CX_ON"

    def update(self, current_state, pressure, altitude, vertical_velocity):
        '''Update the state of the mission based on sensor readings'''
        if current_state == "LAUNCH_PAD":
            if pressure < launch_site_pressure and altitude > 0:
                self.ascent_counter += 1
                if self.ascent_counter > 4:
                    self.current_state = "ASCENT"
                    self.ascent_counter = 0
            else:
                self.ascent_counter = 0

        elif current_state == "ASCENT":
            if vertical_velocity < 0.0001 and vertical_velocity > -0.0001:
                self.apogee_counter += 1
                if self.apogee_counter > 4:
                    self.current_state = "APOGEE"
                    self.maximum_height = altitude
                    self.apogee_counter = 0
            else:
                self.apogee_counter = 0

        elif current_state == "APOGEE":
            if vertical_velocity < 0:
                self.descent_counter += 1
                if self.descent_counter > 4:
                    self.current_state = "DESCENT"
                    self.descent_counter = 0
            else:
                self.descent_counter = 0

        elif current_state == "DESCENT":
            if altitude <= 0.8*self.maximum_height:
                self.probe_release_counter += 1
                if self.probe_release_counter > 4:
                    self.current_state = "PROBE_RELEASE"
                    self.probe_release_counter = 0
            else:
                self.probe_release_counter = 0

        elif current_state == "PROBE_RELEASE":
            if altitude <= self.zero_calibration + 2:
                self.payload_release_counter += 1
                if self.payload_release_counter > 4:
                    self.current_state = "PAYLOAD_RELEASE"
                    self.payload_release_counter = 0
            else:
                self.payload_release_counter = 0

        elif current_state == "PAYLOAD_RELEASE":
            if vertical_velocity < 0.0001 and vertical_velocity > -0.0001:
                self.landed_counter += 1
                if self.landed_counter > 4:
                    self.current_state = "LANDED"
                    global mode
                    mode = "CX_OFF"
            else:
                self.landed_counter = 0


def read_values():
    '''Read the values from the sensors'''
    global pressure, altitude, vertical_velocity
    # Read the values from the sensors and update the variables
    # pressure = read_pressure_sensor()
    # altitude = read_altitude_sensor()
    # vertical_velocity = read_accelerometer_sensor()
    pass


fsw_state = FswState()

while mode == "CX_ON":
    read_values()
    fsw_state.update(fsw_state.current_state, pressure,
                     altitude, vertical_velocity)
