import numpy as np
import matplotlib.pyplot as plt

def triwave(t, amplitude, n_waves):
    per = np.size(t) / n_waves
    fract_t = t / per
    tri = 2 * (np.abs(2 * (fract_t - np.floor(fract_t + .5))) - .5)
    return tri

def gen_pwm(amplitude, n_pulses, periods, length, plot=True):
    t_tri = np.arange(length)
    t = np.linspace(0, periods, length, endpoint=False)

    sin = np.sin(t * np.pi * 2) * amplitude
    # tri = -triwave(t_tri, 2, n_pulses)/2+.5
    tri = -triwave(t_tri, 2, n_pulses)
    pwm = np.uint64(np.greater(sin, tri))

    if plot:
        plt.figure(figsize=(7, 5))

        plt.subplot(2, 1, 1)
        plt.plot(t, sin, label="Sine Wave")
        plt.plot(t, tri, label="Triangle Wave")
        plt.grid(True)
        # plt.xticks(np.linspace(0, periods, 9))
        plt.xlim((0, periods))
        plt.ylim((-1.05, 1.05))

        plt.subplot(2, 1, 2)
        plt.plot(t, pwm, label="PWM", color='green')
        plt.title("Generated PWM")
        plt.grid(True)
        # plt.xticks(np.linspace(0, periods, 9))
        plt.xlim((0, periods))
        plt.ylim((-.05, 1.05))
        plt.tight_layout()
        plt.show()

    return pwm

gen_pwm(0.75, (3*2+.5)*3, 3, 4000)