#!/usr/bin/python3

import pygame
import requests
import json
import time
import serial
import atexit

DEBUG = True
JOYSTICK_NUM = 0
SPEED_AXIS = 1
STEER_AXIS = 3
BRAKE_BUTTON = 4
AXIS_BUTTON = 5

def data_send(s, speed: float, steer: float) -> None:
    #print("Sending data" +  str(speed) + str(steer))
    #print(int.to_bytes(int(speed * 127), length=1, signed=True))
    #print(int.to_bytes(int(steer * 127), length=1, signed=True))
    data = b'c' + int.to_bytes(int(speed * 127), length=1, signed=True) + b'\n' + b'r' + int.to_bytes(int(steer * 127), length=1, signed=True) + b'\n'
    s.write(data)
    #print("Data" + str(list(data)))

def joystick_read_axis(joystick: pygame.joystick.Joystick, axis: int) -> float:
    return joystick.get_axis(axis)

def joystick_read_button(joystick: pygame.joystick.Joystick, button: int) -> float:
    return joystick.get_button(button)

def joystick_setup() -> list[pygame.joystick.Joystick]:
    pygame.init()
    pygame.joystick.init()
    print(f'Found {pygame.joystick.get_count()} joysticks on ports')
    joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
    for joystick in joysticks:
        joystick.init()
        print(f'Joystick {joystick.get_id()}: {joystick.get_name()}, axis: {joystick.get_numaxes()}')
    return joysticks

def main():
    with serial.Serial("COM3", 9600, timeout=10) as s:
        joysticks = joystick_setup()
        print(f'Found {pygame.joystick.get_count()} joysticks on ports')
        while True:
            pygame.event.pump()
            steer = joystick_read_axis(joysticks[JOYSTICK_NUM], STEER_AXIS)
            brake = joystick_read_button(joysticks[JOYSTICK_NUM], BRAKE_BUTTON)
            axis = joystick_read_button(joysticks[JOYSTICK_NUM], AXIS_BUTTON)
            if brake == 0:
                speed = - joystick_read_axis(joysticks[JOYSTICK_NUM], SPEED_AXIS)
                if speed < 0:
                    speed = 0
                elif axis:
                    speed = 0.3 + (speed * 0.7)
            else:
                speed = 0.0
            steer = round(steer, 3)
            speed = round(speed, 3)
            if DEBUG:
                print(f'Speed: {speed}, Steer: {steer}')
            data_send(s, speed, steer)
            #print("Got data: " + s.readline().decode('utf-8', errors='ignore'))
            time.sleep(0.2)

if __name__ == '__main__':
    main()

