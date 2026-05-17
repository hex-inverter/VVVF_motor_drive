from textwrap import dedent, indent
import numpy as np
import matplotlib.pyplot as plt
import helpers


FILENAME = 'pwm_tab'

PERIOD = 1 << 12
BITS = 8
BASELINE = 0.05
PULSES = (1, 3, 5, 9, 15, 27, 45, 81, 135) # japanese values (135 made up)
PULSES = (1, 3, 5, 9, 15, 25, 41, 67, 109, 177, 299)


def display(pwm):
	pwm1 = helpers.fold4(pwm)
	pwm2 = np.roll(pwm1, pwm1.size // 3)
	t = np.linspace(0, 1, pwm1.size)

	pwm_line = pwm2 - pwm1

	u = helpers.v_avg(pwm_line)

	ax = plt.subplot(311)
	plt.plot(t, pwm1, ds='steps-mid')
	ax = plt.subplot(312, sharex=ax)
	plt.plot(t, pwm2, ds='steps-mid')
	ax = plt.subplot(313, sharex=ax)
	plt.plot(t, pwm_line, t, u, ds='steps-mid')
	plt.tight_layout()
	plt.show()

	a = np.abs(np.fft.rfft(u)) / u.size
	plt.semilogy(a)
	plt.xlim((0, 25))
	plt.ylim((10e-5, 1))
	plt.tight_layout()
	plt.show()


def make_table(period, pulses, bits=8):
	bits1 = (1 << bits) - 1
	tri = helpers.tri_touching(period // 2, 3*pulses)[:period // 4 + 1]
	tri = helpers.tri_normal(period // 2, 3*pulses)[:period // 4 + 1]
	tri2 = helpers.tri_normal(period // 4, 1.5 * pulses)
	tri2 = np.append(tri2, np.sign(tri2[-1]))

	if np.count_nonzero(tri2-tri):
		plt.close()
		plt.plot(tri-tri2)
		plt.show()
		print('AAAAAAAAAAAAAAAA')

	t = np.linspace(0, 0.25, period // 4 + 1, endpoint=True)
	wave = helpers.sin3(t)
	wave = helpers.weird_sin(t)

	lut = np.full_like(t, bits1, dtype=np.int32)
	for a in np.linspace(BASELINE, 1, bits1, endpoint=True):
		lut -= a * wave >= tri


	return lut


def print_c_array(a, name):
	txt = [f'const uint8_t {name}[{a.size}] = {{']

	ln = ['\t']
	for i, val in enumerate(a):
		if not i % 16 and i:
			txt.append(''.join(ln).rstrip())
			ln = ['\t']

		ln.append(f'{val:3d}, ')

	txt.append(''.join(ln).rstrip())
	txt.append('};\n')

	return txt


def main():
	t2 = np.linspace(0, 1, PERIOD, endpoint=False)
	pure = np.sin(2 * np.pi * t2)


	txt = [dedent(f'''\
		#ifndef {FILENAME.upper()}
		#define {FILENAME.upper()}

		#include <inttypes.h>


		#define PERIOD 0x{PERIOD:X}

	''')]


	fig, ax = plt.subplots()
	lines = ax.plot(t2, pure, t2, pure, t2, pure, ds='steps-mid')
	plt.xlim((0, 1))
	plt.pause(0.1)

	for n in PULSES:
		print(f'{n} pulses')

		lut = make_table(PERIOD, n, BITS)

		txt += print_c_array(lut, f'pwm{n}')

		# for i in range(0, 1 << BITS):
		for i in ():
			print(i)
			a = i / ((1 << BITS) - 1) * (1 - BASELINE) + BASELINE
			pwm = i >= lut

			pwm1 = helpers.fold4(pwm)
			pwm2 = np.roll(pwm1, pwm1.size // 3)
			pwm_line = np.roll(pwm2 - pwm1, 7 * pwm1.size // 12)
			u = helpers.v_avg(pwm_line)

			lines[0].set_ydata(a * pure)
			lines[1].set_ydata(u)
			lines[2].set_ydata(10 * (a*pure - u))

			# lines[0].set_ydata(pwm2 - pwm1)
			# lines[1].set_ydata(pwm1 * .5 + .05)
			# lines[2].set_ydata(pwm2 * .5 - .05)

			fig.canvas.draw()
			fig.canvas.flush_events()

			# ---------------------
			# pwm1 = fold2(pwm) # (pwm made from wave2)
			# pwm2 = np.roll(pwm1, pwm1.size // 3)
			# pwm_line = np.roll(pwm2 - pwm1, 7 * pwm1.size // 12)
			# u2 = helpers.v_avg_new(pwm_line)

			# lines[0].set_ydata(10*(a*pure-u2))
			# lines[1].set_ydata(10*(u-u2))

			# fig.canvas.draw()
			# fig.canvas.flush_events()

	plt.close()


	txt.append('#endif\n')
	txt = '\n'.join(txt)

	with open(f'{FILENAME}.c', 'w', encoding='ascii') as f:
		f.write(txt)


if __name__ == '__main__':
	main()
