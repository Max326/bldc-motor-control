#sudo apt-get install python3-tk

import serial
import threading
import tkinter as tk
from tkinter import ttk
import time

# Serial port configuration
SERIAL_PORT = '/dev/ttyUSB0'  # Set the port for your device
BAUDRATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)

# Function to send commands via UART
def send_command(command):
    if ser.is_open:
        ser.write(command.encode())

# Function to continuously read data from STM32 and update the log
def read_from_stm32():
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            update_log(data)
        time.sleep(1)

# Function to update the log window in the GUI
def update_log(message):
    log_textbox.insert(tk.END, message + '\n')
    log_textbox.see(tk.END)

# Motor control functions
def increase_speed():
    send_command('=')

def decrease_speed():
    send_command('-')

def change_direction():
    send_command('d')

def increase_torque():
    send_command(']')

def decrease_torque():
    send_command('[')

def toggle_pot_mode():
    send_command('p')

# Function to create the GUI for motor control
def create_gui():
    root = tk.Tk()
    root.title("Motor Control Panel")

    # Speed control buttons
    ttk.Button(root, text="Increase Speed", command=increase_speed).grid(row=0, column=0, padx=10, pady=10)
    ttk.Button(root, text="Decrease Speed", command=decrease_speed).grid(row=0, column=1, padx=10, pady=10)

    # Torque control buttons (placed before the row containing Change Direction and Toggle Pot Mode)
    ttk.Button(root, text="Increase Torque", command=increase_torque).grid(row=1, column=0, padx=10, pady=10)
    ttk.Button(root, text="Decrease Torque", command=decrease_torque).grid(row=1, column=1, padx=10, pady=10)

    # Direction and Pot Mode buttons
    ttk.Button(root, text="Change Direction", command=change_direction).grid(row=2, column=0, padx=10, pady=10)
    ttk.Button(root, text="Toggle Pot Mode", command=toggle_pot_mode).grid(row=2, column=1, padx=10, pady=10)

    # Log display window with increased width
    global log_textbox
    log_textbox = tk.Text(root, height=10, width=60)  # Increased width to 60
    log_textbox.grid(row=3, column=0, columnspan=2, padx=10, pady=10)

    # Start reading data from STM32 in a separate thread
    threading.Thread(target=read_from_stm32, daemon=True).start()

    root.mainloop()

if __name__ == "__main__":
    create_gui()
