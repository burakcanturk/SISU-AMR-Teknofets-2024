from serial import Serial
from time import sleep
import struct

RECEIVING_RIGHT_MOTOR_SPEED_ROW = 0
RECEIVING_LEFT_MOTOR_SPEED_ROW = 1
RECEIVING_RIGHT_MOTOR_DIRECTIFRONT_ROW = 2
RECEIVING_LEFT_MOTOR_DIRECTIFRONT_ROW = 3
RECEIVING_RIGHT_MOTOR_SPEED_MODE_ROW = 4
RECEIVING_LEFT_MOTOR_SPEED_MODE_ROW = 5
RECEIVING_RIGHT_MOTOR_STOP_ROW = 6
RECEIVING_LEFT_MOTOR_STOP_ROW = 7
RECEIVING_RIGHT_LINEER_SPEED_ROW = 8
RECEIVING_LEFT_LINEER_SPEED_ROW = 9
RECEIVING_RIGHT_LINEER_DIRECTIFRONT_ROW = 10
RECEIVING_LEFT_LINEER_DIRECTIFRONT_ROW = 11
RECEIVING_RIGHT_LANTERN_VAL_ROW = 12
RECEIVING_LEFT_LANTERN_VAL_ROW = 13
RECEIVING_STRIP_LED_VAL_ROW = 14
RECEIVING_BUZZER_VAL_ROW = 15
RECEIVING_DIRECTIFRONT_RESET_ROW = 16

SENDING_RIGHT_MOTOR_VAL_ROW = 128
SENDING_LEFT_MOTOR_VAL_ROW = 129
SENDING_FRONT_LEFT_LINE_VAL_ROW = 130
SENDING_FRONT_MIDDLE_LINE_VAL_ROW = 131
SENDING_FRONT_RIGHT_LINE_VAL_ROW = 132
SENDING_CENTER_LEFT_LINE_VAL_ROW = 133
SENDING_CENTER_MIDDLE_LINE_VAL_ROW = 134
SENDING_CENTER_RIGHT_LINE_VAL_ROW = 135
SENDING_LEFT_DISTANCE_VAL_ROW = 136
SENDING_MIDDLE_DISTANCE_VAL_ROW = 137
SENDING_RIGHT_DISTANCE_VAL_ROW = 138
SENDING_VOLTAGE_VAL_ROW = 139
SENDING_CURRENT_VAL_ROW = 140
SENDING_MASS_VAL_ROW = 141
SENDING_UPPER_DISTANCE_VAL_ROW = 142
SENDING_REM_CHRG_UPPER_BOX_ROW = 143
SENDING_ROLL_VAL_ROW = 144
SENDING_PITCH_VAL_ROW = 145
SENDING_YAW_VAL_ROW = 146
SENDING_LDR_VAL_ROW = 147

SERVO_MIN = 500
SERVO_MAX = 2500
SERVO_STOP = 500

MOTOR_SPEED_MODE_NORMAL = 0
MOTOR_SPEED_MODE_SLOW = 1
MOTOR_SPEED_MODE_FAST = 2

LINEER_HEIGHT_NORMAL = 0
LINEER_HEIGHT_LOW = 1
LINEER_HEIGHT_HIGH = 2

amrVehicleReceiving = {
    "right_motor_val": SERVO_STOP,
    "left_motor_val": SERVO_STOP,
    "right_motor_direction": 0,
    "left_motor_direction": 0,
    "right_motor_speed_mode": MOTOR_SPEED_MODE_NORMAL,
    "left_motor_speed_mode": MOTOR_SPEED_MODE_NORMAL,
    "right_motor_stop": 0,
    "left_motor_stop": 0,
    "right_lineer_val": 0,
    "left_lineer_val": 0,
    "right_lineer_direction": LINEER_HEIGHT_NORMAL,
    "left_lineer_direction": LINEER_HEIGHT_NORMAL,
    "right_lantern_val": 0,
    "left_lantern_val": 0,
    "strip_led_val": 0,
    "buzzer_val": 0,
    "direction_reset_val": 0
}

amrVehicleSending =  {
    "right_motor_speed": 0.0,
    "left_motor_speed": 0.0,
    "front_left_cizgi_val": 0,
    "front_orta_cizgi_val": 0,
    "front_right_cizgi_val": 0,
    "center_left_cizgi_val": 0,
    "center_middle_cizgi_val": 0,
    "center_right_cizgi_val": 0,
    "left_distance_val": 0.0,
    "orta_distance_val": 0.0,
    "right_distance_val": 0.0,
    "voltage": 0.0,
    "current": 0.0,
    "mass": 0.0,
    "upper_distance_val": 0,
    "rem_chrg_upper_box": 0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "ldr_val": 0
}

def constrain(num, min_, max_):
    if num < min_:
        return min_
    elif num > max_:
        return max_
    else:
        return num

class amrVehicle(Serial):

    def __init__(self, port, baud):
        self.__init__(port, baud)
        sleep(3)

    def setRightMotorSpeed(self, val):
        amrVehicleReceiving["right_motor_val"] = constrain(val, SERVO_MIN, SERVO_MAX)
        self.write(RECEIVING_RIGHT_MOTOR_SPEED_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["right_motor_val"].to_bytes(2, "little"))
        self.read(1)

    def setLeftMotorSpeed(self, val):
        amrVehicleReceiving["left_motor_val"] = constrain(val, SERVO_MIN, SERVO_MAX)
        self.write(RECEIVING_LEFT_MOTOR_SPEED_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["left_motor_val"].to_bytes(2, "little"))
        self.read(1)

    def setRightMotorDirection(self, val):
        amrVehicleReceiving["right_motor_direction"] = bool(val)
        self.write(RECEIVING_RIGHT_MOTOR_DIRECTIFRONT_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["right_motor_direction"].to_bytes(1, "little"))
        self.read(1)

    def setLeftMotorDirection(self, val):
        amrVehicleReceiving["left_motor_direction"] = bool(val)
        self.write(RECEIVING_LEFT_MOTOR_DIRECTIFRONT_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["left_motor_direction"].to_bytes(1, "little"))
        self.read(1)

    def setRightMotorSpeedMode(self, val):
        amrVehicleReceiving["right_motor_speed_mode"] = constrain(val, 0, 2)
        self.write(RECEIVING_RIGHT_MOTOR_SPEED_MODE_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["right_motor_speed_mode"].to_bytes(1, "little"))
        self.read(1)

    def setLeftMotorSpeedMode(self, val):
        amrVehicleReceiving["left_motor_speed_mode"] = constrain(val, 0, 2)
        self.write(RECEIVING_LEFT_MOTOR_SPEED_MODE_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["left_motor_speed_mode"].to_bytes(1, "little"))
        self.read(1)

    def setRightMotorStop(self, val):
        amrVehicleReceiving["right_motor_stop"] = bool(val)
        self.write(RECEIVING_RIGHT_MOTOR_STOP_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["right_motor_stop"].to_bytes(1, "little"))
        self.read(1)

    def setLeftMotorStop(self, val):
        amrVehicleReceiving["left_motor_stop"] = bool(val)
        self.write(RECEIVING_LEFT_MOTOR_STOP_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["left_motor_stop"].to_bytes(1, "little"))
        self.read(1)

    def setRightLineerSpeed(self, val):
        amrVehicleReceiving["right_lineer_val"] = constrain(val, 0, 255)
        self.write(RECEIVING_RIGHT_LINEER_SPEED_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["right_lineer_val"].to_bytes(1, "little"))
        self.read(1)

    def setLeftLineerSpeed(self, val):
        amrVehicleReceiving["left_lineer_val"] = constrain(val, 0, 255)
        self.write(RECEIVING_LEFT_LINEER_SPEED_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["left_lineer_val"].to_bytes(1, "little"))
        self.read(1)

    def setRightLineerDirection(self, val):
        amrVehicleReceiving["right_lineer_direction"] = constrain(val, 0, 2)
        self.write(RECEIVING_RIGHT_LINEER_DIRECTIFRONT_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["right_lineer_direction"].to_bytes(1, "little"))
        self.read(1)

    def setLeftLineerDirection(self, val):
        amrVehicleReceiving["left_lineer_direction"] = constrain(val, 0, 2)
        self.write(RECEIVING_LEFT_LINEER_DIRECTIFRONT_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["left_lineer_direction"].to_bytes(1, "little"))
        self.read(1)

    def setRightLantern(self, val):
        amrVehicleReceiving["right_lantern_val"] = bool(val)
        self.write(RECEIVING_RIGHT_LANTERN_VAL_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["right_lantern_val"].to_bytes(1, "little"))
        self.read(1)

    def setLeftLantern(self, val):
        amrVehicleReceiving["left_lantern_val"] = bool(val)
        self.write(RECEIVING_LEFT_LANTERN_VAL_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["left_lantern_val"].to_bytes(1, "little"))
        self.read(1)

    def setStripLed(self, val):
        amrVehicleReceiving["strip_led_val"] = val
        self.write(RECEIVING_STRIP_LED_VAL_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["strip_led_val"].to_bytes(1, "little"))
        self.read(1)

    def setBuzzer(self, val):
        amrVehicleReceiving["buzzer_val"] = bool(val)
        self.write(RECEIVING_BUZZER_VAL_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["buzzer_val"].to_bytes(1, "little"))
        self.read(1)

    def setDirectionReset(self, val):
        amrVehicleReceiving["direction_reset_val"] = bool(val)
        self.write(RECEIVING_DIRECTIFRONT_RESET_ROW.to_bytes(1, "little"))
        self.write(amrVehicleReceiving["direction_reset_val"].to_bytes(1, "little"))
        self.read(1)

    #---------------------------------------------------------------------------

    def getRightMotorSpeed(self):
        self.write(SENDING_RIGHT_MOTOR_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["right_motor_speed"] = round(struct.unpack("f", self.read(4))[0], 2)
        return amrVehicleSending["right_motor_speed"]

    def getLeftMotorSpeed(self):
        self.write(SENDING_LEFT_MOTOR_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["left_motor_speed"] = round(struct.unpack("f", self.read(4))[0], 2)
        return amrVehicleSending["left_motor_speed"]

    def getFrontLeftLineVal(self):
        self.write(SENDING_FRONT_LEFT_LINE_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["front_left_cizgi_val"] = int.from_bytes(self.read(2), "little")
        return amrVehicleSending["front_left_cizgi_val"]

    def getFrontMiddleLineVal(self):
        self.write(SENDING_FRONT_MIDDLE_LINE_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["front_orta_cizgi_val"] = int.from_bytes(self.read(2), "little")
        return amrVehicleSending["front_orta_cizgi_val"]

    def getFrontRightLineVal(self):
        self.write(SENDING_FRONT_RIGHT_LINE_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["front_right_cizgi_val"] = int.from_bytes(self.read(2), "little")
        return amrVehicleSending["front_right_cizgi_val"]

    def getCenterLeftLineVal(self):
        self.write(SENDING_CENTER_LEFT_LINE_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["center_left_cizgi_val"] = int.from_bytes(self.read(2), "little")
        return amrVehicleSending["center_left_cizgi_val"]

    def getCenterMiddleLineVal(self):
        self.write(SENDING_CENTER_MIDDLE_LINE_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["center_middle_cizgi_val"] = int.from_bytes(self.read(2), "little")
        return amrVehicleSending["center_middle_cizgi_val"]

    def getCenterRightLineVal(self):
        self.write(SENDING_CENTER_RIGHT_LINE_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["center_right_cizgi_val"] = int.from_bytes(self.read(2), "little")
        return amrVehicleSending["center_right_cizgi_val"]

    def getLeftDistanceVal(self):
        self.write(SENDING_LEFT_DISTANCE_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["left_distance_val"] = round(struct.unpack("f", self.read(4))[0], 2)
        return amrVehicleSending["left_distance_val"]

    def getMiddleDistanceVal(self):
        self.write(SENDING_MIDDLE_DISTANCE_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["orta_distance_val"] = round(struct.unpack("f", self.read(4))[0], 2)
        return amrVehicleSending["orta_distance_val"]

    def getRightDistanceVal(self):
        self.write(SENDING_RIGHT_DISTANCE_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["right_distance_val"] = round(struct.unpack("f", self.read(4))[0], 2)
        return amrVehicleSending["right_distance_val"]

    def getVoltage(self):
        self.write(SENDING_VOLTAGE_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["voltage"] = round(struct.unpack("f", self.read(4))[0], 2)
        return amrVehicleSending["voltage"]

    def getCurrent(self):
        self.write(SENDING_CURRENT_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["current"] = round(struct.unpack("f", self.read(4))[0], 2)
        return amrVehicleSending["current"]

    def getMass(self):
        self.write(SENDING_MASS_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["mass"] = round(struct.unpack("f", self.read(4))[0], 2)
        return amrVehicleSending["mass"]

    def getUpperDistanceVal(self):
        self.write(SENDING_UPPER_DISTANCE_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["upper_distance_val"] = int.from_bytes(self.read(1), "little")
        return amrVehicleSending["upper_distance_val"]

    def getRemChrgUpperBoxVal(self):
        self.write(SENDING_REM_CHRG_UPPER_BOX_ROW.to_bytes(1, "little"))
        amrVehicleSending["rem_chrg_upper_box"] = int.from_bytes(self.read(2), "little")
        return amrVehicleSending["rem_chrg_upper_box"]

    def getRollVal(self):
        self.write(SENDING_ROLL_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["roll"] = round(struct.unpack("f", self.read(4))[0], 2)
        return amrVehicleSending["roll"]

    def getPitchVal(self):
        self.write(SENDING_PITCH_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["pitch"] = round(struct.unpack("f", self.read(4))[0], 2)
        return amrVehicleSending["pitch"]

    def getYawVal(self):
        self.write(SENDING_YAW_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["yaw"] = round(struct.unpack("f", self.read(4))[0], 2)
        return amrVehicleSending["yaw"]

    def getLdrVal(self):
        self.write(SENDING_LDR_VAL_ROW.to_bytes(1, "little"))
        amrVehicleSending["ldr_val"] = int.from_bytes(self.read(2), "little")
        return amrVehicleSending["ldr_val"]
