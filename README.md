This is project Klika.

(DoorNob in translation.)

Main components:
  STM32WB15CC - MCU
  TMC2209 - stepper motor driver
  NEMA17 stepper motor

UART
  Communication between MCU and TMC2209.
  MCU receives information about current drivers status and sends commands to controll the motor.

BLE
  Communication between MCU and smartphone.
  MCU receives commands to lock or unlock the door.
