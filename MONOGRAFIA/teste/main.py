import PySimpleGUI as sg
import serial
import subprocess

class SerialDataLogger:
    def __init__(self, port='COM4', baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.filepath = fr"{subprocess.check_output('echo %userprofile%', shell=True).decode().strip()}\Desktop\data.txt"

        self.layout = [[sg.Button('Start Reading', key='-START-'), sg.Button('Stop Reading', key='-STOP-')]]
        self.window = sg.Window('Serial Data Logger', self.layout)

    def write_to_file(self, data):
        with open(self.filepath, "ab") as f:
            f.write(data)

    def read_serial(self):
        ser = serial.Serial(self.port, self.baudrate)
        while True:
            data = ser.readline()
            self.write_to_file(data)

    def start_reading(self):
        self.read_serial()

    def stop_reading(self):
        ser = serial.Serial(self.port, self.baudrate)
        ser.close()
        self.window.close()
        subprocess.Popen([r'explorer', self.filepath])

    def run(self):
        while True:
            event, values = self.window.read()
            if event == sg.WINDOW_CLOSED:
                break
            elif event == '-START-':
                self.start_reading()
            elif event == '-STOP-':
                self.stop_reading()

if __name__ == '__main__':
    logger = SerialDataLogger()
    logger.run()