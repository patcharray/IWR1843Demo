import tkinter as tk
from tkinter import ttk

from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure

import read_vital_signs
import queue
import time
import serial
import threading
import numpy as np
from collections import deque

N = 90          # number of samples visible on the plot
WINDOW = 180    # maximal time interval of the plot

data_q = queue.Queue()
times = deque(maxlen=N)
br = deque(maxlen=N)
hr = deque(maxlen=N)

median_breathing = np.zeros(21)
median_heartbeat = np.zeros(21)

start = time.monotonic()

is_running = True

def reader(port):
    i = 0
    try:
        _reader(port, i)
    except Exception as exc:
        print(f"reader thread stopped: {exc!r}")


def _reader(port, i):
    with serial.Serial(port, baudrate=read_vital_signs.DATA_PORT_BAUD, timeout=1) as ser:
        while True:
            values = read_vital_signs.read_frame(ser)
            if values and None not in values:
                if (-5 <= values[0] <= -1) or (-5 <= values[1] <= -1):
                    continue                  # the board is still settling

                if values[0] == -6:
                    median_breathing[i] = 0           # if board reports -6, it means that person is not detected, 
                else:                                 # so output 0 on breathing rate/heartbeat
                    median_breathing[i] = values[0]

                if values[1] == -6:
                    median_heartbeat[i] = 0
                else:
                    median_heartbeat[i] = values[1]

                i += 1

                if i == 21:                           
                    breathing_output = np.median(median_breathing)  # in order to suppress outliers, store 21 most recent
                    heartbeat_output = np.median(median_heartbeat)  # values and output the median
                                                                    # the buffer is cleared afterwards, so the plot receives
                                                                    # new value every 21 * 100ms = 2.1s
                    data_q.put((time.monotonic() - start, breathing_output, heartbeat_output))
                    i = 0


def update_plot():
    while not data_q.empty():
        t, b, h = data_q.get_nowait()
        times.append(t)
        br.append(b)
        hr.append(h)

    while times and times[-1] - times[0] > WINDOW: # if deques have more than 180 values, pop the oldest until only 180 are present
        times.popleft()
        br.popleft()
        hr.popleft()

    br_line.set_data(times, br)
    hr_line.set_data(times, hr)

    for point in (breathing_rate, heartrate):
        point.relim()
        point.autoscale_view(scalex=False)
    
    t_now = times[-1] if times else 0
    breathing_rate.set_xlim(max(0, t_now - WINDOW), max(WINDOW, t_now)) # time axis is always 180 seconds long

    canvas.draw_idle()

def running():
    if is_running:
        update_plot()
    root.after(100, running)

root = tk.Tk()
root.wm_title("Vital Signs Detection Demo")
root.rowconfigure(0, weight=1)
root.columnconfigure(0, weight=1)

content = ttk.Frame(root)
content.rowconfigure(0, weight=8)
content.rowconfigure(1, weight=1)
content.columnconfigure(0, weight=1)
content.columnconfigure(1, weight=1)
content.columnconfigure(2, weight=1)
content.columnconfigure(3, weight=1)
content.columnconfigure(4, weight=1)
content.grid(column=0, row=0, sticky="nsew")

fig = Figure(figsize=(5, 6), dpi=100)
rates = fig.subplots(2, 1, sharex=True)
breathing_rate = rates[0]
(br_line, ) = breathing_rate.plot([], [])
breathing_rate.set_ylabel('Breathing Rate')
breathing_rate.set_yticks(np.arange(0, 60.1, 12))

heartrate = rates[1]
(hr_line, ) = heartrate.plot([], [])
heartrate.set_ylabel('Heartrate')
heartrate.set_xlabel('Time/s')
heartrate.set_yticks(np.arange(0, 120.1, 60), minor=True)

canvas = FigureCanvasTkAgg(fig, master=content) 
canvas.draw()
canvas.get_tk_widget().grid(column=0, row=0, columnspan=5, sticky="nsew")

toolbar = NavigationToolbar2Tk(canvas, content, pack_toolbar=False)
toolbar.update()
toolbar.grid(column=0, row=1, columnspan=2)


def on_key_press(event):
    print("you pressed {}".format(event.key))
    key_press_handler(event, canvas, toolbar)

canvas.mpl_connect("key_press_event", on_key_press)


def _stop():
    global is_running           # stops the plotting process
    is_running = False

def _reset():
    global start                # resets the plot and wipes all queues clean
    start = time.monotonic()

    with data_q.mutex:
        data_q.queue.clear()
    times.clear()          
    br.clear()
    hr.clear()

def _quit():
    root.quit()                 # stops mainloop
    root.destroy()              # this is necessary on Windows to prevent
                                # Fatal Python Error: PyEval_RestoreThread: NULL tstate


button_stop = tk.Button(master=content, text="Stop", command=_stop)
button_stop.grid(column=2, row=1, sticky="nsew", pady=5, padx=5)

button_reset = tk.Button(master=content, text="Reset", command=_reset)
button_reset.grid(column=3, row=1, sticky="nsew", pady=5, padx=5)

button_quit = tk.Button(master=content, text="Quit", command=_quit)
button_quit.grid(column=4, row=1, sticky="nsew", pady=5, padx=5)

root.update()
root.minsize(root.winfo_width(), root.winfo_height())

# Execute readout on a different thread
threading.Thread(target=reader, args=("/dev/ttyACM1",), daemon=True).start()


root.after(100, running)
tk.mainloop()
# If you put root.destroy() here, it will cause an error if the window is
# closed with the window manager.
    