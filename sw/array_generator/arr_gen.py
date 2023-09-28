import json
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# Constants for each wave
LENGTH = 2**11
# PULSES = (15, 23, 31, 43, 61, 83, 127, 167) # primes close to HEF4752
# PULSES = (15, 21, 30, 42, 60, 84, 120, 168) # original numer of pulses
PULSES = (1, 3, 5, 9, 15, 27, 45, 81) # japanese values (quite different TBH)

def triwave(t, amplitude, n_waves):
    per = np.size(t) / n_waves
    fract_t = t / per
    tri = 2 * (np.abs(2 * (fract_t - np.floor(fract_t + .5))) - .5)
    return tri


def gen_pwm(amplitude, n_pulses, length, plot=False):
    t_tri = np.arange(length)
    t = np.linspace(0, 1, length, endpoint=False)

    sin = np.sin(t * np.pi) * amplitude
    tri = -triwave(t_tri, 2, n_pulses)/2+.5
    pwm = np.uint64(np.greater(sin, tri))

    if plot:
        plt.figure(figsize=(7, 5))

        plt.subplot(2, 1, 1)
        plt.plot(t, sin, label="Sine Wave")
        plt.plot(t, tri, label="Triangle Wave")
        plt.grid(True)
        plt.xticks(np.linspace(0, 1, 9))
        plt.xlim((0, 1))
        plt.ylim((-1.05, 1.05))

        plt.subplot(2, 1, 2)
        plt.plot(t, pwm, label="PWM", color='green')
        plt.title("Generated PWM")
        plt.grid(True)
        plt.xticks(np.linspace(0, 1, 9))
        plt.xlim((0, 1))
        plt.ylim((-.05, 1.05))
        plt.tight_layout()
        plt.show()

    return pwm


def postprocess(pwm):
    # generate a complete period (including endpoint)
    pwm = np.append(pwm, -pwm)
    pwm = np.append(pwm, pwm[0])
    # and a corresponding time
    t = np.append(t, t[-1] + t[1] + t)
    t = np.append(t, t[-1] + t[1])

    # phase PWM
    pwm_ph = [
        np.copy(pwm),
        np.roll(pwm, int(np.around(np.size(pwm) / 3))),
        np.roll(pwm, -int(np.around(np.size(pwm) / 3))),
    ]

    # phase to phase PWM
    pwm_line = []
    # phase to phase voltage
    v_line = []
    for i, wave in enumerate(pwm_ph):
        # the phase before
        prev = pwm_ph[(i - 1) % 3]

        line = wave - prev
        pwm_line.append(line)

        voltage = np.cumsum(line)
        average = np.average(voltage)
        print(f'offset: {average}')

        v_line.append(voltage - average)

    # plt.figure(figsize=(7, 5))
    # for wave in pwm_line:
    #     plt.plot(t, wave)
    # plt.show()

    plt.figure(figsize=(7, 5))
    for wave in v_line:
        plt.plot(t, wave)
    plt.show()


mod_table = np.zeros((len(PULSES), LENGTH), dtype=np.uint64)
for i, n_pulses in enumerate(PULSES):
    for voltage in np.linspace(0, 1, 256)[1:]:
        mod_table[i] += gen_pwm(voltage, n_pulses, LENGTH)

    mod_table[i] = 255 - mod_table[i]

for table in mod_table:
    plt.figure(figsize=(7, 5))
    plt.plot(table)
    # plt.show()


for i, table in enumerate(mod_table):
    with open(f'pwm_{PULSES[i]}.txt', 'w', encoding='utf-8') as f:
        f.write(f'const uint8_t pwm_{PULSES[i]}[{table.size}] = {{')
        for j, num in enumerate(table):
            if j % 16 == 0:
                f.write('\n\t')
            f.write(f'{num: 4},')
        f.write('\n};')

with open('mod_table.json', 'w', encoding='utf-8') as f:
    json.dump(mod_table.tolist(), f)#, indent='\t')