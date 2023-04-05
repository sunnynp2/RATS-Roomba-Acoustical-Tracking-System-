from scipy.fftpack import fft, fftfreq
import numpy as np
from matplotlib import pyplot as plt
import csv

'''
this code runs a fast fourier transform of an audio recording titled
"Audio_Sample_For_Python.csv" stored in the same folder
The purpose of this code is to verify that the speaker and microphones
are both emitting the expected frequency of 1000 hz.and to verify that
we are able to get accurate phase shift information. 
'''


SAMPLE_RATE = 71428.6  # Hertz
DURATION = 35e-3  # Seconds



with open("Audio_Sample_For_Python.csv") as f:
	reader = csv.reader(f)
	row_list = [row for row in reader]
    
row_list.pop(0)
audio_sample = np.squeeze(np.array(row_list))
audio_sample = audio_sample.astype(int)
plt.figure().set_figheight(1)
plt.plot(audio_sample)

plt.show()
print(audio_sample)

'''
because the audio data doesn't record time, we have to create
an array that tracks time on the x axis. This needs to be accurate 
to the sampling frequency of the microphones or else our FFT calculations 
would be incorrect. 
'''

time = []

for i in range(0, 2499):
    time.append(i * (35e-3/2500))
time = np.array(time)


#mean = audio_sample / np.sum(audio_sample)

'''
Here we are normalizing the sound wave so it is centered on y=0
'''

amplitude = (audio_sample.max() - audio_sample.min()) / 2
audio_sample = audio_sample - (audio_sample.min() + amplitude)
normalized_tone = np.int16((audio_sample / audio_sample.max()) * 32767)
plt.plot(time, normalized_tone)
plt.show()

'''
in the following code we use the FFT functions in SciPy to generate the
real and imaginary components of the fourier transform so we can then 
use the peak to calculate phase shifts
'''

# Number of samples in normalized_tone
N = 2499

yf = fft(normalized_tone)
xf = fftfreq(N, 1 / SAMPLE_RATE)

plt.plot(xf, np.real(yf))
plt.xlim(-3000, 3000)
plt.title("Real Part")
plt.xlabel("Frequency")
plt.ylabel("Amplitude")
plt.show()

plt.plot(xf, np.imag(yf))
plt.xlim(-3000,3000)
plt.title("Imaginary Part")
plt.xlabel("Frequency")
plt.ylabel("Amplitude")
plt.show()

real = np.real(yf)
imag = np.imag(yf)

mag = imag**2 + real**2
plt.plot(xf, mag)
plt.xlim(-3000,3000)
plt.title("magnitude")
plt.xlabel("Frequency")
plt.ylabel("Amplitude")
plt.show()

mindex = np.where(mag == np.amax(mag))
print('index of max amplitude: ', mindex)
print("frequency of max amplitude: " , xf[mindex])
print("phase shift is: ", np.arctan(imag[mindex]/real[mindex]) * 360 / (2 * np.pi))
print()