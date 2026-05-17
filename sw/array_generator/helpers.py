import numpy as np


def tri_touching(num, freq, minw=None):
	points = np.empty(freq + 2, dtype=np.int32)
	half_wave = num / freq / 2
	points[1:-1] = np.int32(np.round(np.linspace(half_wave, num - half_wave, freq, endpoint=True)))
	points[0] = 0
	points[-1] = num

	val = np.full_like(points, 1)
	val[1::2] = -1
	val[0] = 0
	val[-1] = 0

	tri = np.full(num, np.nan)
	for x0, y0, x1, y1 in zip(points, val, points[1:], val[1:]):
		tri[x0:x1] = np.linspace(y0, y1, x1 - x0, endpoint=False)

	return tri


def tri_normal(num, freq):
	freq /= 2
	t = np.linspace(-1 / freq / 4, 1 - 1/freq/4, num, endpoint=False)
	return 4 * np.abs(t * freq - np.floor(t * freq + .5)) - 1


def sin3(x):
	return np.sqrt(1/3) * (2 * np.sin(2 * np.pi * x) + np.sin(2 * np.pi * 3 * x) / 3)


def sapwm(x):
	x %= 1

	wave = np.where(x < 1/3, np.sin(2 * np.pi * x), np.sin(2 * np.pi * (x + 1/3)))
	wave = np.where(x > 0.5, 0, x)

	return wave


def weird_sin(x):
	x %= 1

	wave = np.sqrt(3) * np.sin(x * 2 * np.pi)
	wave = np.where(np.logical_and(1/12 < x, x <= 1/4), np.sin((x + 1/12) * 2 * np.pi), wave)
	wave = np.where(np.logical_and(1/4 < x, x <= 5/12), np.sin((x - 1/12) * 2 * np.pi), wave)
	wave = np.where(np.logical_and(7/12 < x, x <= 3/4), np.sin((x + 1/12) * 2 * np.pi), wave)
	wave = np.where(np.logical_and(3/4 < x, x <= 11/12), np.sin((x - 1/12) * 2 * np.pi), wave)

	return wave


def fold2(sig):
	return np.append(sig, 1 - sig)


def fold2_pwm(sig):
	sig = np.append(sig, 1 - sig)
	return np.append(sig, sig[0])


def fold4(sig):
	sig = np.append(sig, np.flip(sig[1:-1]))
	return fold2(sig)


def v_avg(pwm):
	u = np.cumsum(pwm) / pwm.size * np.pi * 2
	u -= np.average(u)
	return np.roll(u, - u.size // 4)


# def v_avg_new(pwm):
# 	# u = np.cumsum(np.append(pwm, pwm[0])) / (pwm.size - 0) * np.pi * 2
# 	u = np.trapz(pwm, dx=1 / pwm.size)
# 	u -= np.average(u)
# 	return np.roll(u[:-1], - u.size // 4 - 1)


PULSES = (1, 3, 5, 9, 15, 25, 41, 67, 109, 177, 299)
