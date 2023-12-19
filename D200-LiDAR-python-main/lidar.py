import serial
import csv
from collections import deque
import copy

import matplotlib.pyplot as plt 
from math import cos, sin, pi
import math

class Circle:
    def __init__(self):
        self.packet_list = []

    def add(self, new_packet):
        if len(self.packet_list) > 0:
            last_angle = self.packet_list[-1].end_angle
        else:
            last_angle = 0
        if new_packet.end_angle is not None \
            and new_packet.end_angle > last_angle:
                self.packet_list.append(new_packet)
                return True
        return False

    def __repr__(self) -> str:
        return f"Circle object with {len(self.packet_list)} packets"

    def points(self):
        for packet in self.packet_list:
            for datapoint in packet.datapoints:
                yield datapoint

    def export_csv(self, path):
        with open(path, "w", newline='') as f:
            fieldnames = ['timestamp', 'angle', 'distance', 'signal']
            writer = csv.DictWriter(f, fieldnames, extrasaction='ignore')
            writer.writeheader()
            for p in self.points():
                writer.writerow(p.to_dict())

class Packet:
    def __init__(self, raw_data: bytearray):
        self.datapoints = []
        self.radar_speed = None
        self.start_angle = None
        self.end_angle = None
        self.complete = False
        self.datapoints_appended = 0
        self.raw_data = raw_data
        if raw_data[0:1] == Lidar.START_BYTE and raw_data[1:2] == Lidar.VER_LEN:
            self._decode()
        
    def __str__(self) -> str:
        datapoints_string = '\n'.join(d.__repr__() for d in self.datapoints)
        return f"Packet object with data: speed: {self.radar_speed}RPS, \
                angle: {self.start_angle}-{self.end_angle}deg\n{datapoints_string}"

    def _decode(self):
        self.radar_speed = int.from_bytes(self.raw_data[2:4], 'little')
        self.start_angle = int.from_bytes(self.raw_data[4:6], 'little') * 0.01
        self.end_angle = int.from_bytes(self.raw_data[42:44], 'little') * 0.01
        self.timestamp = int.from_bytes(self.raw_data[44:46], 'little')
        if self.timestamp > 30000:
            raise ValueError
        for i in range(6, 42, 3):
            three_bytes = self.raw_data[i:i+3]
            distance = int.from_bytes(three_bytes[0:2], 'little')
            signal = int.from_bytes(three_bytes[2:3], 'little')
            new_datapoint = Datapoint(distance, signal, self.timestamp)
            self.datapoints.append(new_datapoint)
            self.datapoints_appended += 1
        step = (self.end_angle - self.start_angle) / (len(self.datapoints) - 1)
        for i, d in enumerate(self.datapoints):
            d.angle = round(self.start_angle + step * i, 2)
        self.complete = True


class Datapoint:
    def __init__(self, distance, signal, timestamp):
        self.angle = None
        self.distance = distance
        self.signal_strength = signal
        self.timestamp = timestamp

    def __repr__(self) -> str:
        return f"Datapoint object: angle= {self.angle}, distance= {self.distance}, signal= {self.signal_strength}, timestamp= {self.timestamp}"

    def to_dict(self):
        return {'timestamp' : self.timestamp,
                'angle' : self.angle,
                'distance' : self.distance,
                'signal' : self.signal_strength}

class Lidar:
    #first two bytes of packet are constant according to: https://www.waveshare.com/wiki/D200_LiDAR_Kit#Communication_Protocol
    START_BYTE = b'\x54'
    VER_LEN = b'\x2c'

    def __init__(self, port: str, baudrate=230400):
        self.port = port
        self.baudrate = baudrate
    
    def capture_circle(self, path=None):
        with serial.Serial() as ser:
            ser.baudrate = self.baudrate
            ser.port = self.port
            circles = []
            current_bytes = bytearray()
            current_circle = Circle()
            circles.append(current_circle)
            last_two_bytes_received = deque([b'\x00', b'\x00'],2)
            ser.open()
            while len(circles) <= 2:
                #read one byte
                x = ser.read()
                #add it to the queue to watch for new packet starting sequence
                last_two_bytes_received.append(x)
                #after starting sequence is detected
                if (last_two_bytes_received[0], last_two_bytes_received[1]) == (Lidar.START_BYTE, Lidar.VER_LEN):
                    #and there are already some bytes recorded
                    if len(current_bytes) > 0:
                        #compose them into a Packet Object
                        try:
                            new_packet = Packet(current_bytes)
                        except ValueError:
                            continue
                        #and if it's complete
                        if new_packet.complete:
                            #try to add it into a Circle Object
                            #it may fail, meaning the Circle is full
                            if not current_circle.add(new_packet):
                                #if the current circle is full, create new Circle Object and add the packet there
                                current_circle = Circle()
                                circles.append(current_circle)
                                current_circle.add(new_packet)
                        #clear the array to collect new bytes
                        current_bytes = bytearray()
                        #add starting byte
                        current_bytes += last_two_bytes_received[0]
                #add a byte to the current bytearray
                current_bytes += x
       
        return circles[1]


if __name__ == "__main__":
    lidar = Lidar('COM4')

    x = []
    y = []

    for _ in range(10):
        data = lidar.capture_circle()

        filtered_data = []

        for p in data.packet_list:
            for d in p.datapoints:
                if d.distance > 0:
                    filtered_data.append(d)

        for i, lm in enumerate(filtered_data):
            if 360 > lm.angle > 0 and lm.distance < 2500:
                x.append(lm.distance / 10 * sin(math.radians(float(lm.angle))))
                y.append(lm.distance / 10 * cos(math.radians(float(lm.angle))))

    plt.scatter(x, y) 
    plt.show()
    
