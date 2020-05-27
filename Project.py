import string
import sys
import os
import serial
#import heartpy as hp
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from tkinter import *
from tkinter.ttk import *
from tkinter import  ttk

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []
arr = []
count = 0

ser = serial.Serial()

def read():
    a=''
    while True:
        if ser is None:
            continue
        try:
            val = ser.readline().decode("utf-8")
            return val
        except:
            print(" ")


def animate(i, xs, ys):
    global count
    val = read()
    arr.append(val)
    count = count + 1
    print(count)
    print(val)

    # Add x and y to lists
    xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    ys.append(val)

    # Limit x and y lists to 20 items
    xs = xs[-20:]
    ys = ys[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Heart Monitor')
    plt.ylabel('Heart sensor data')

def write():
    global port
    port = port.get()
    print(port)
    
    global brate
    brate = brate.get()
    print(brate)
    
    global srate
    srate = srate.get()
    print(brate)
    
    ser.port = port
    ser.baudrate = int(brate)
    ser.bytesize = serial.EIGHTBITS
    ser.partiy = serial.PARITY_NONE
    ser.timeout = 2
    ser.open()

    ser.write(srate.encode())

    ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=50)

    plt.show()

window = Tk()
window.title("Project")
window.geometry('250x100')


lbl = Label(window, text="Select Port Number")
lbl.grid(column=0, row=0)
port = Entry(window)
port.grid(column=1, row=0)

lbl = Label(window, text="Select Baudrate")
lbl.grid(column=0, row=1)
brate = Entry(window)
brate.grid(column=1, row=1)

lbl = Label(window, text="Select Sample Rate")
lbl.grid(column=0, row=2)
srate = Entry(window)
srate.grid(column=1, row=2)

btn = Button(window, text="Start", command=write)
btn.grid(column=1, row=3)

window.mainloop()



