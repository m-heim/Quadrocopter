#!/usr/bin/python3

import pygame
import time
import sys
import serial


DEBUG = True
JOYSTICK_NUM = 0
SPEED_AXIS = 1
STEER_AXIS = 3
YAW_AXIS = 0
ROLL_AXIS = 2
BRAKE_BUTTON = 4
AXIS_BUTTON = 5
SETTING_BUTTON = 3

def data_send(s, speed: float, steer: float,  yaw: float, roll: float) -> None:
    #print("Sending data" +  str(speed) + str(steer))
    #print(int.to_bytes(int(speed * 127), length=1, signed=True))
    #print(int.to_bytes(int(steer * 127), length=1, signed=True))
    data = b'\x06' + str(hex(int(speed * 127))).encode('utf-8') + b',' + str(hex(int(steer * 127))).encode('utf-8') + b',' + str(hex(int(yaw * 127))).encode('utf-8') + b',' + str(hex(int(roll * 127))).encode('utf-8') + b'\0'
    s.write(data)
    #print("Data" + str(list(data)))

def setting_send(s) -> None:
    print("Sending setting")
    data = b's' + b'\n'
    s.write(data)
    print("Sending setting ok")

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
    if len(sys.argv) != 2:
        print("Usage: python data.py <serial_port>")
        sys.exit(1)

    with serial.Serial(sys.argv[1], 115200, timeout=10) as s:
        joysticks = joystick_setup()
        print(f'Found {pygame.joystick.get_count()} joysticks on ports')
        while True:
            pygame.event.pump()
            steer = -joystick_read_axis(joysticks[JOYSTICK_NUM], STEER_AXIS)
            yaw = joystick_read_axis(joysticks[JOYSTICK_NUM], YAW_AXIS)
            roll = joystick_read_axis(joysticks[JOYSTICK_NUM], ROLL_AXIS)
            brake = joystick_read_button(joysticks[JOYSTICK_NUM], BRAKE_BUTTON)
            axis = joystick_read_button(joysticks[JOYSTICK_NUM], AXIS_BUTTON)
            setting = joystick_read_button(joysticks[JOYSTICK_NUM], SETTING_BUTTON)
            if brake == 0:
                speed = - joystick_read_axis(joysticks[JOYSTICK_NUM], SPEED_AXIS)
                if axis:
                    speed = ((speed + 1) * 0.5)
                #elif speed < 0:
                #    speed = 0
            else:
                speed = 0.0
            steer = round(steer, 3)
            speed = round(speed, 3)
            if DEBUG:
                print(f'Speed: {speed}, Steer: {steer}, Yaw: {yaw}, Roll: {roll}')
            data_send(s, speed, steer, yaw, roll)
            if setting:
                setting_send(s)
            #for _ in range(1):
                #try:
                    #print("")
                    #print("Got data: " + s.readline().decode('utf-8', errors='ignore'))
                #except:
                    #break
            time.sleep(0.2)

if __name__ == '__main__':
    main()

