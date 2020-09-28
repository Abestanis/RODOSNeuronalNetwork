import os
import select
import socket
import time
from math import degrees, radians
from argparse import ArgumentParser
from contextlib import nullcontext
from enum import IntEnum
from random import random
from struct import pack, unpack, calcsize, error as StructError
from threading import Thread, Lock
from tkinter import Tk, Entry, Button, mainloop, Label

from serial import Serial

from motorCtrl import MotorController


class ThreadSaveSerial(Serial):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._lock = Lock()

    def read(self, size=1):
        with self._lock:
            return super().read(size)

    def write(self, data):
        with self._lock:
            return super().write(data)


class SocketSerial:
    def __init__(self):
        super().__init__()
        self._waitForConnection()

    def read(self, size=1):
        if select.select([self.clientSocket], [], [], 0)[0]:
            return self.clientSocket.recv(size)
        return b''

    def write(self, data):
        self.clientSocket.sendall(data)

    def close(self):
        self.clientSocket.close()

    def _waitForConnection(self):
        serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serverSocket.bind(("localhost", 9999))
        serverSocket.listen(1)
        print('Waiting for connection... ', end='')
        self.clientSocket, _ = serverSocket.accept()
        print('OK')
        serverSocket.close()


class Controller:
    # noinspection SpellCheckingInspection
    DATA_FORMAT = r'<IqB?dddddddddddddddc'
    DATA_LENGTH = calcsize(DATA_FORMAT)
    SECOND = 1e+9
    _training = False
    _currentTrainingState = None
    _currentTrainingTime = 0
    _secondTrainingTarget = None

    class Autopilot(IntEnum):
        INVALID = -1
        NONE = 0
        PID = 1
        NN = 2

    def __init__(self, _port, _baudrate, motorController, randomTargets, replayTargets):
        super().__init__()
        # self.serial = Serial(port, baudrate, timeout=1)
        # self.serial = ThreadSaveSerial(port, baudrate, timeout=0)
        self.serial = SocketSerial()
        self._outputFile = None
        self._profilerOutputFile = None
        self._motorController = motorController
        self._randomTargets = randomTargets
        self._replayTargets = replayTargets
        self._goodTrainingSteps = 0
        self._trainingCmdTimestamp = 0
        self.running = True
        self.buffer = b''
        self.hadUnknownCommand = False
        self.COMMANDS = {
            b'p': self.handle_print,
            b'd': self.handle_data,
            b'r': self.handle_profilerData,
        }

    def handleControlTarget(self):
        if self._randomTargets:
            while self.running:
                target = (0, random() * 45 - 17, 0)
                self.sendTarget(*target)
                print(f'Target: {target}')
                time.sleep(random() * 5 + 2)
                speed = (self._randomWindTunnelMotorSpeed(), self._randomWindTunnelMotorSpeed())
                self.sendWindTunnelSpeed(*speed)
                print(f'Wind tunnel speed: {speed}')
                time.sleep(random() * 5 + 2)
        elif self._replayTargets is not None:
            DATA_FORMAT = '<dddddd'
            CHUNK_SIZE = calcsize(DATA_FORMAT)
            with open(self._replayTargets, 'rb') as replayFile:
                while self.running:
                    chunk = replayFile.read(CHUNK_SIZE)
                    if len(chunk) < CHUNK_SIZE:
                        break
                    try:
                        sleepTime, targetYaw, targetPitch, targetRoll, speedLeft, speedRight = \
                            unpack(DATA_FORMAT, chunk)
                    except StructError as error:
                        print(f'Unable to decode data replay entry: {error}')
                        continue
                    time.sleep(sleepTime)
                    self.sendTarget(targetYaw, targetPitch, targetRoll)
                    self.sendWindTunnelSpeed(speedLeft, speedRight)

    def showInputWindow(self):
        window = Tk()
        window.title('Ground-Station')
        infoText = Label(window, text="""Commands:

q: Quit.
p: Send ping.
t: {yaw pitch roll}: Set target orientation [°].
s: {elevator ailerons rudder}: Control the servos [°].
a: {0=None | 1=PID | 2=NN}: Select controller.
w: {left right}: Select wind tunnel motor speed [%].
""")
        infoText.pack()
        textField = Entry(window)
        textField.pack()
        textField.focus_set()
        sendButton = Button(window, text="SEND", width=10,
                            command=lambda: self.handleInput(window, textField.get()))
        sendButton.pack()
        mainloop()

    def handleInput(self, window, commandText):
        commandParts = commandText.split(' ')
        if len(commandParts) < 1:
            return
        command = commandParts[0]
        commandParts = commandParts[1:]
        if command == 'q':
            self.running = False
            self.serial.close()
            window.quit()
        elif command == 'p':
            self.serial.write(b'p')
        elif command == 's':
            if len(commandParts) != 3:
                print(f'Invalid number of servo values: {len(commandParts)}')
                return
            try:
                commandParts = [float(number) for number in commandParts]
            except ValueError as error:
                print(f'Invalid servo value: {error}')
                return
            self.sendServoCommand(*commandParts)
        elif command == 't':
            if len(commandParts) != 3:
                print(f'Invalid number of orientation target values: {len(commandParts)}')
                return
            try:
                commandParts = [float(number) for number in commandParts]
            except ValueError as error:
                print(f'Invalid orientation target value: {error}')
                return
            self.sendTarget(*commandParts)
        elif command == 'a':
            if len(commandParts) != 1:
                print(f'Invalid number of arguments: {len(commandParts)}')
                return
            try:
                selectedAutopilot = Controller.Autopilot(int(commandParts[0]))
            except ValueError as error:
                print(f'Invalid autopilot selection: {error}')
                return
            self.serial.write(pack(r'<cB', b'a', selectedAutopilot))
        elif command == 'w':
            if len(commandParts) != 2:
                print(f'Invalid number of arguments: {len(commandParts)}')
                return
            try:
                commandParts = [float(number) for number in commandParts]
            except ValueError as error:
                print(f'Invalid wind tunnel motor speed value: {error}')
                return
            self.sendWindTunnelSpeed(*commandParts)
        else:
            print(f'Unknown command "{command}"')

    def handle_print(self, buffer):
        buffer, lengthBytes = self._readBytes(buffer, 4)
        try:
            length = unpack('I', lengthBytes)[0]
            print(self.serial.read(length).decode('utf-8', errors='replace'))
        except StructError as error:
            print(f'Unable to decode data length: {error}')
        return buffer

    def handle_data(self, buffer):
        buffer, data = self._readBytes(buffer, self.DATA_LENGTH)
        if data[-1:] != b'E':
            print('Invalid Data')
        elif self._outputFile is None:
            self._parseAndPrintData(data[:-1], self)
        else:
            self._outputFile.write(data[:-1])
        return buffer

    def handle_profilerData(self, buffer):
        DATA_FORMAT = r'<Q'
        buffer, data = self._readBytes(buffer, calcsize(DATA_FORMAT))
        if self._profilerOutputFile is not None:
            self._profilerOutputFile.write(data)
        else:
            try:
                controllerExecutionTimeNs = unpack(DATA_FORMAT, data)[0]
                print(f'Controller execution time: {controllerExecutionTimeNs} ns')
            except StructError as error:
                print(f'Unable to decode data controller execution time: {error}')
        return buffer

    def main(self, outputDir, profilerOutput):
        outputPath = None
        if outputDir is not None:
            os.makedirs(outputDir, exist_ok=True)
            outputPath = os.path.join(outputDir, time.strftime('%Y.%m.%d %H-%M-%S.bin'))
            print(f'Writing data to {outputPath}...')
        inputThread = Thread(target=self.showInputWindow)
        controlTargetThread = Thread(target=self.handleControlTarget)
        with nullcontext() if outputPath is None else open(outputPath, 'wb') as self._outputFile, \
                nullcontext() if profilerOutput is None else \
                        open(profilerOutput, 'wb') as self._profilerOutputFile:
            inputThread.start()
            controlTargetThread.start()
            while self.running:
                self.handleOneCommand()
        inputThread.join()
        controlTargetThread.join()

    def handleOneCommand(self):
        while len(self.buffer) < 1:
            self.buffer += self.serial.read(2048)
        command = self.buffer[:1]
        self.buffer = self.buffer[1:]
        try:
            handler = self.COMMANDS[command]
        except KeyError:
            if command == b'\0':
                return
            if not self.hadUnknownCommand or True:
                print(f'Unknown command received {command}')
            self.hadUnknownCommand = True
        else:
            # noinspection PyArgumentList
            self.buffer = handler(self.buffer)
            self.hadUnknownCommand = False

    def sendServoCommand(self, elevatorServo, aileronsServo, rudderServo):
        data = pack(r'<cddd', b's', elevatorServo, aileronsServo, rudderServo)
        self.serial.write(data)

    def sendTarget(self, yaw, pitch, roll):
        data = pack(r'<cddd', b't', radians(yaw), radians(pitch), radians(roll))
        self.serial.write(data)

    def sendWindTunnelSpeed(self, leftSpeed, rightSpeed):
        if self._motorController is not None:
            self._motorController.setSpeedLeft(leftSpeed)
            self._motorController.setSpeedRight(rightSpeed)
        else:
            data = pack(r'<cdd', b'w', leftSpeed, rightSpeed)
            self.serial.write(data)

    def resetTrainingState(self):
        self._training = True
        self._goodTrainingSteps = 0
        self._currentTrainingState = None
        self._currentTrainingTime = 0
        self.sendTarget(0, random() * 45 - 17, 0)
        self.sendWindTunnelSpeed(
            self._randomWindTunnelMotorSpeed(), self._randomWindTunnelMotorSpeed())
        self._secondTrainingTarget = (
            self._randomWindTunnelMotorSpeed(), self._randomWindTunnelMotorSpeed())
        while self._currentTrainingState is None:
            self.handleOneCommand()
        self._trainingCmdTimestamp = self._currentTrainingTime
        return self._currentTrainingState

    def executeTrainingStep(self, nnOutput):
        self.sendServoCommand(nnOutput[0], nnOutput[1], nnOutput[2])
        self._currentTrainingState = None
        while self._currentTrainingState is None:
            self.handleOneCommand()
        targetError = degrees(sum((
            abs(self._currentTrainingState[9] - self._currentTrainingState[6]),
            abs(self._currentTrainingState[10] - self._currentTrainingState[7]),
            abs(self._currentTrainingState[11] - self._currentTrainingState[8]),
        )))
        nsPassedSinceCommand = self._currentTrainingTime - self._trainingCmdTimestamp
        reward = (10 - targetError + max(0.0, 2 - (nsPassedSinceCommand / self.SECOND)))
        if targetError < 2:
            self._goodTrainingSteps += 1
        else:
            self._goodTrainingSteps = 0
        done = self._goodTrainingSteps >= 20

        if done and self._secondTrainingTarget is not None:
            done = False
            self._goodTrainingSteps = 0
            self.sendWindTunnelSpeed(*self._secondTrainingTarget)
            self._secondTrainingTarget = None
            self._trainingCmdTimestamp = self._currentTrainingTime
        return self._currentTrainingState, reward, done

    @classmethod
    def analyzeDataFile(cls, path):
        with open(path, 'rb') as dataFile:
            while True:
                chunk = dataFile.read(cls.DATA_LENGTH - 1)
                if chunk == b'':
                    break
                cls._parseAndPrintData(chunk)

    @classmethod
    def _parseAndPrintData(cls, dataBytes, self=None):
        try:
            data = unpack(cls.DATA_FORMAT[:-1], dataBytes)
        except StructError as error:
            print(f'[Warn] Failed to parse data chunk: {error}')
            return
        if self is not None and self._training:
            if not data[3]:  # If not sensor ready
                return
            self._currentTrainingTime = data[1]
            self._currentTrainingState = [
                data[7], data[8], data[9],  # Gyroscope
                data[4], data[5], data[6],  # Accelerometer
                data[15], data[13], data[14],  # Orientation
                data[18], data[16], data[17],  # Target orientation
            ]
            return
        try:
            selectedAutopilot = Controller.Autopilot(data[2])
        except ValueError:
            selectedAutopilot = Controller.Autopilot.INVALID
        print(f"i={data[0]}, time={data[1]}, selectedAutopilot={selectedAutopilot!s}, "
              f"sensorsReady={data[3]}, accelerometer (x={data[4]:.{2}f}, y={data[5]:.{2}f}, "
              f"z={data[6]:.{2}f}), gyroscope (x={data[7]:.{2}f}, y={data[8]:.{2}f}, "
              f"z={data[9]:.{2}f}), servos (elevator={data[10]:.{2}f}, ailerons={data[11]:.{2}f}, "
              f"rudder={data[12]:.{2}f}), yaw={degrees(data[13]):.{2}f}, "
              f"pitch={degrees(data[14]):.{2}f}, roll={degrees(data[15]):.{2}f}, "
              f"targetYaw={degrees(data[16]):.{2}f}, targetPitch={degrees(data[17]):.{2}f}, "
              f"targetRoll={degrees(data[18]):.{2}f}")

    def _readBytes(self, buffer, size):
        missingBytes = max(0, size - len(buffer))
        byteBuffer = buffer[:size - missingBytes]
        buffer = buffer[size - missingBytes:]
        while missingBytes > 0:
            byteBuffer += self.serial.read(missingBytes)
            missingBytes = size - len(byteBuffer)
        return buffer, byteBuffer

    @staticmethod
    def _randomWindTunnelMotorSpeed():
        return random() * 50 + 50


if __name__ == '__main__':
    _parser = ArgumentParser(description='Communicates the plane controller.')
    _parser.add_argument('--analyze', help='Analyze a data file.')
    _parser.add_argument('-p', '--port', help='The serial port the plane is connected to.',
                         default='COM4')
    _parser.add_argument('-b', '--baudrate', help='The baudrate to use for the serial connection.',
                         type=int, default=115200)
    _parser.add_argument('-o', '--outputDir', help='The directory to write the received data '
                                                   'from the plane controller to.')
    _parser.add_argument('--profilerOutput', default=None,
                         help='The path to a file to store the profiler data.')
    _parser.add_argument('-m', '--motorCtrlPort', default=None,
                         help='The serial port the wind-tunnel-motor controller is connected to.')
    _parser.add_argument('--leftMotorAddress', default=None, type=int,
                         help='The address of the left wind-tunnel-motor.')
    _parser.add_argument('--rightMotorAddress', default=None, type=int,
                         help='The address of the right wind-tunnel-motor.')
    _parser.add_argument('-r', '--randomTargets', default=False, action='store_true',
                         help='Weather or not to send random targets to the controller.')
    _parser.add_argument('-t', '--replayTargets', default=None,
                         help='Path to a file that contains targets to the controller.')
    _parser.add_argument('--train', help='Train a neuronal network.')
    _parser.add_argument('--trainOutput', help='The path to store the trained model.',
                         default=os.path.join('models', 'model.tflite'))
    _args = _parser.parse_args()
    if _args.analyze is not None:
        Controller.analyzeDataFile(_args.analyze)
    else:
        _motorController = None
        if _args.motorCtrlPort is not None:
            _motorController = MotorController(
                _args.motorCtrlPort, _args.leftMotorAddress, _args.rightMotorAddress)
        controller = Controller(_args.port, _args.baudrate, _motorController, _args.randomTargets,
                                _args.replayTargets)
        if _args.train is not None:
            from model import qTraining

            qTraining(_args.train, _args.trainOutput, controller)
        else:
            controller.main(_args.outputDir, _args.profilerOutput)
