import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

##PID controller gains
Kp_raw = 250.0
Ki_raw = 500.0
Kd_raw = 2.5

output_scale = 100.0 / 65535.0

##Effective Controller Transfer Function C(s) = (Kp + Ki/s + Kd*s) * Scale
num_c = [Kd_raw * output_scale, Kp_raw * output_scale, Ki_raw * output_scale]
den_c = [1, 0]
sys_c = signal.TransferFunction(num_c, den_c)

K_plant = 600.0 

##motor lag
tau_motor = 0.03

##Plant Transfer Function P(s) = K_plant / (s * (tau_motor*s + 1))
##Models the drone's rotational dynamics and integration from w to angle
num_p = [K_plant]
den_p = [tau_motor, 1, 0]
sys_p = signal.TransferFunction(num_p, den_p)

##Open Loop Transfer Function L(s) = C(s) * P(s)
##Combined system response used for frequency domain analysis (Bode/Nyquist)
num_l = np.convolve(num_c, num_p)
den_l = np.convolve(den_c, den_p)
sys_l = signal.TransferFunction(num_l, den_l)

w = np.logspace(-1, 3, 1000) * 2 * np.pi

##Bode Plot
w, mag, phase = signal.bode(sys_l, w)
freq_hz = w / (2 * np.pi)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
plt.subplots_adjust(hspace=0.4)

##Magnitude
ax1.semilogx(freq_hz, mag, color='blue')
ax1.set_title('Bode Plot - Open Loop L(s)')
ax1.set_ylabel('Magnitude (dB)')
ax1.grid(True, which="both", ls="-", alpha=0.5)
ax1.axhline(0, color='red', linestyle='--')

##Phase
ax2.semilogx(freq_hz, phase, color='blue')
ax2.set_ylabel('Phase (degrees)')
ax2.set_xlabel('Frequency (Hz)')
ax2.grid(True, which="both", ls="-", alpha=0.5)
ax2.axhline(-180, color='red', linestyle='--')

##Isolate Phase Margin & Crossover
crossover_idx = np.abs(mag).argmin()
crossover_freq = freq_hz[crossover_idx]
phase_margin = 180 + phase[crossover_idx]

ax1.plot(crossover_freq, mag[crossover_idx], 'ro')
ax1.text(crossover_freq, 5, f'Cross: {crossover_freq:.1f} Hz', verticalalignment='bottom')

ax2.plot(crossover_freq, phase[crossover_idx], 'ro')
ax2.text(crossover_freq, phase[crossover_idx] + 10, f'PM: {phase_margin:.1f} deg', verticalalignment='bottom')

plt.savefig('bode_plot.png', dpi=100)
plt.close()

##Nyquist Plot
w, H = signal.freqresp(sys_l, w)

plt.figure(figsize=(8, 8))
plt.plot(H.real, H.imag, 'b', label='L(jw)')
plt.plot(H.real, -H.imag, 'b--', alpha=0.3)
plt.plot(-1, 0, 'rx', markersize=10, label='Critical Point (-1, 0)')

theta = np.linspace(0, 2*np.pi, 100)
plt.plot(np.cos(theta), np.sin(theta), 'k:', alpha=0.3)

plt.title('Nyquist Plot')
plt.xlabel('Real Axis')
plt.ylabel('Imaginary Axis')
plt.grid(True)
plt.legend()
plt.axis('equal')

plt.xlim(-2.5, 1)
plt.ylim(-1.5, 1.5)
plt.savefig('nyquist_plot.png', dpi=100)
plt.close()
print(f"PM:{phase_margin:.2f}")
print(f"Cross:{crossover_freq:.2f}")
