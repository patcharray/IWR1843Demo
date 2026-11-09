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
import sys
import numpy as np
from collections import deque
from datetime import datetime

N = 90                      # number of samples visible on the plot
WINDOW = N * 2              # maximal time interval of the plot
SETTLE_TIME = 18            # settle time of the board
AVG_COUNT = 15              # interval of log updates averaging
MA_SAMPLES = 5              # number of samples used in moving average

data_q = queue.Queue()
times = deque(maxlen=N)     # time axis
br = deque(maxlen=N)        # breathing rate
hr = deque(maxlen=N)        # heartrate
br_inter = deque(maxlen=N)
hr_inter = deque(maxlen=N)

median_breathing = np.zeros(21)
median_heartbeat = np.zeros(21)

start = time.monotonic()

is_running = False

settling = True

new_entries = 0

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM1"

def reader(port):
    i = 0
    try:
        _reader(port, i)
    except Exception as exc:
        print(f"reader thread stopped: {exc!r}")

def end_settling():
    global settling, start, is_running
    settling = False
    is_running = True
    start = time.monotonic()
    settling_label.grid_remove()

def _reader(port, i):
    with serial.Serial(port, baudrate=read_vital_signs.DATA_PORT_BAUD, timeout=1) as ser:
        while True:
            values = read_vital_signs.read_frame(ser)
            if values and None not in values:
                if settling:
                    continue

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
    global new_entries
    while not data_q.empty():
        t, b, h = data_q.get_nowait()
        times.append(t)
        br_inter.append(b)
        hr_inter.append(h)
        moving_average()
        new_entries +=1
        if new_entries == AVG_COUNT:
            update_log()
            new_entries = 0
        if br_inter:
            status_label.config(text=f"Breathing: {br[-1]:.1f}     Heartbeat: {hr[-1]:.1f}")

    while times and times[-1] - times[0] > WINDOW: # if deques have more than 180 values, pop the oldest until only 180 are present
        times.popleft()
        br_inter.popleft()
        hr_inter.popleft()
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


def update_log():
    br_avg = np.mean(list(br_inter)[-AVG_COUNT:])
    hr_avg = np.mean(list(hr_inter)[-AVG_COUNT:])

    log_of_values.insert("end", f"{datetime.now():%H:%M:%S}   BR {br_avg:5.1f}  HR{hr_avg:5.1f}")
    log_of_values.see("end")


def moving_average():
    br_ma = (np.mean(list(br_inter)[-MA_SAMPLES:]))
    hr_ma = (np.mean(list(hr_inter)[-MA_SAMPLES:]))

    br.append(br_ma)
    hr.append(hr_ma)

def running():
    if is_running:
        update_plot()
    root.after(100, running)

root = tk.Tk()
root.wm_title("Vital Signs Detection Demo")
root.rowconfigure(0, weight=1)
root.columnconfigure(0, weight=1)

content = ttk.Frame(root)
content.rowconfigure(0, weight=1)       # plot
content.rowconfigure(1, weight=1)
content.rowconfigure(2, weight=0)       # plot toolbar
content.rowconfigure(3, weight=0)       # status label
content.rowconfigure(4, weight=0)       # buttons
content.rowconfigure(5, weight=0)       # settling label
content.columnconfigure(0, weight=3)    
content.columnconfigure(1, weight=3)
content.columnconfigure(2, weight=3)
content.columnconfigure(3, weight=1)
content.columnconfigure(4, weight=0)
content.grid(column=0, row=0, sticky="nsew")

fig = Figure(figsize=(5, 6), dpi=100)
rates = fig.subplots(2, 1, sharex=True)
breathing_rate = rates[0]
(br_line, ) = breathing_rate.plot([], [])
breathing_rate.set_ylabel('Breathing Rate')
breathing_rate.set_yticks(np.arange(5, 40.1, 5))

heartrate = rates[1]
(hr_line, ) = heartrate.plot([], [])
heartrate.set_ylabel('Heartrate')
heartrate.set_xlabel('Time/s')
heartrate.set_yticks(np.arange(0, 120.1, 60), minor=True)

canvas = FigureCanvasTkAgg(fig, master=content) 
canvas.draw()
canvas.get_tk_widget().grid(column=0, row=0, columnspan=3, rowspan=2, sticky="nsew")

toolbar = NavigationToolbar2Tk(canvas, content, pack_toolbar=False)
toolbar.update()
toolbar.grid(column=0, row=2, columnspan=3)

status_label = ttk.Label(content, text=f"Breathing: -      Heartbeat: -")
status_label.grid(column=0, row=3, columnspan=3, pady=5)

settling_label = ttk.Label(content, text=f"Board is settling...")
settling_label.grid(column=0, row=5, columnspan=5, pady=5)

log_of_values = tk.Listbox(master=content, height=5)
log_of_values.grid(column=3, row=0, rowspan=5, sticky="nsew")

sbar = tk.Scrollbar(master=content, orient="vertical", command=log_of_values.yview)
sbar.grid(column=4, row=0, rowspan=5, sticky="ns")
log_of_values.config(yscrollcommand=sbar.set)

def on_key_press(event):
    print("you pressed {}".format(event.key))
    key_press_handler(event, canvas, toolbar)

canvas.mpl_connect("key_press_event", on_key_press)


def _stop():
    global is_running           # stops the plotting process
    is_running = False

def _reset():
    global start , is_running, settling, new_entries      # resets the plot and wipes all queues clean
    if settling:
        return
    start = time.monotonic()

    with data_q.mutex:
        data_q.queue.clear()
    times.clear()
    br_inter.clear()
    hr_inter.clear()
    br.clear()
    hr.clear()
    is_running = True
    new_entries = 0
    update_plot()


def _quit():
    root.quit()                 # stops mainloop
    root.destroy()              # this is necessary on Windows to prevent
                                # Fatal Python Error: PyEval_RestoreThread: NULL tstate


button_stop = tk.Button(master=content, text="Stop", command=_stop)
button_stop.grid(column=0, row=4, sticky="nsew", pady=5, padx=5)

button_reset = tk.Button(master=content, text="Reset", command=_reset)
button_reset.grid(column=1, row=4, sticky="nsew", pady=5, padx=5)

button_quit = tk.Button(master=content, text="Quit", command=_quit)
button_quit.grid(column=2, row=4, sticky="nsew", pady=5, padx=5)

root.update()
root.minsize(root.winfo_width() + 150, root.winfo_height())

# Execute readout on a different thread
threading.Thread(target=reader, args=(port,), daemon=True).start()

root.after(SETTLE_TIME * 1000, end_settling)
root.after(100, running)
tk.mainloop()
# If you put root.destroy() here, it will cause an error if the window is
# closed with the window manager.
    