import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

with open('mod_table.json', 'r', encoding='utf-8') as f:
    raw = json.load(f)

mod_table = np.array(raw)

mod_table[0] = 255 - mod_table[0]

pwm = mod_table[0]
threshold = 128

fig, ax = plt.subplots(figsize=(5, 5))
plt.subplots_adjust(left=0.25, bottom=0.25)

# Set axis limits
ax.set_xlim(0, pwm.size)
ax.set_ylim(-.05, 1.05)

# Create axes for the sliders
ax_slider_y = plt.axes([0.1, 0.25, 0.03, 0.65], facecolor='lightgoldenrodyellow')
ax_slider_x = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor='lightgoldenrodyellow')

# Create the sliders
slider_y = Slider(ax_slider_y, 'Y', 0, 255, valstep=1, orientation='vertical')
slider_x = Slider(ax_slider_x, 'X', 0, 7, valstep=1)

def refresh():
    global threshold
    global pwm
    wave = np.int64(np.greater(threshold, pwm))
    wave = np.append(wave, -wave)
    wave = np.append(wave, wave[0])

    voltage = np.cumsum(np.roll(wave, wave.size // 4))

    ax.clear()
    ax.plot(wave, drawstyle='steps-mid')
    ax.plot(voltage, drawstyle='steps-mid')

def swap_table(i):
    global pwm
    pwm = mod_table[i]
    refresh()

def update_threshold(val):
    global threshold
    threshold = val
    refresh()

slider_x.on_changed(swap_table)
slider_y.on_changed(update_threshold)

plt.show()
