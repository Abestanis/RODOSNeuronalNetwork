import struct

from serial import Serial


class MotorController:
    MODBUS_CMD_WRITE = 6
    MB_REG_LFr = 8502

    def __init__(self, comPort, motorAddrLeft, motorAddrRight):
        self.serial = Serial(comPort, 19200)
        self._motorAddrLeft = motorAddrLeft
        self._motorAddrRight = motorAddrRight

    def setSpeedLeft(self, speed):
        return self.setSpeed(self._motorAddrLeft, speed)

    def setSpeedRight(self, speed):
        return self.setSpeed(self._motorAddrRight, speed)

    def setSpeed(self, deviceAddr, speed):
        if speed < 0 or speed > 100:
            return False

        # - Register LFr holds frequency applied to the motor in 0.1Hz
        # - maximum frequency is 50 Hz -> max. register value is 500
        # - 100% = 50Hz = 1500 rpm
        return self.writeRegister(deviceAddr, self.MB_REG_LFr, speed * 10 // 2)

    def writeRegister(self, deviceAddr, regAddr, value):
        sendBuffer = struct.pack('>BBHH', deviceAddr, self.MODBUS_CMD_WRITE, regAddr, value)
        sendBuffer += struct.pack('>H', self.crc16(sendBuffer))

        self.serial.write(sendBuffer)
        receiveBuffer = self.serial.read(len(sendBuffer))
        return sendBuffer == receiveBuffer

    @staticmethod
    def crc16(buffer):
        crc = 0xFFFF
        for byte in buffer:
            crc ^= byte
        for i in range(8):
            if crc & 0x01:
                crc = (crc >> 1) & 0x7FFF
                crc ^= 0xA001
            else:
                crc = (crc >> 1) & 0x7FFF
        return crc
