% Define parameters
Fs = 1000;             % Sampling frequency (Hz)
T = 1;                 % Duration of the signal (s)
t = 0:1/Fs:T-1/Fs;     % Time vector
f_noise = 50;          % Frequency of the noise (Hz)
filter_freq = 200;     % Frequency to filter out (Hz)

% Generate random noise signal
noise_signal = randn(1, length(t));

% Perform FFT on the signal
fft_signal = fft(noise_signal);

% Reconstruct the signal without using ifft
reconstructed_signal = zeros(size(noise_signal));
for k = 1:length(fft_signal)
    if abs(k - Fs/2) > filter_freq
        reconstructed_signal = reconstructed_signal + fft_signal(k) * exp(2i*pi*(k-1)*t);
    end
end

% Calculate the power spectrum of the original and reconstructed signal
power_spectrum_original = abs(fft_signal).^2 / length(t);
power_spectrum_reconstructed = abs(fft(reconstructed_signal)).^2 / length(t);

% Create a 4x4 subplot
subplot(2, 2, 1);
plot(t, noise_signal);
title('Original Signal');
xlabel('Time (s)');
ylabel('Amplitude');

subplot(2, 2, 2);
plot(t, real(reconstructed_signal));
title('Reconstructed Signal');
xlabel('Time (s)');
ylabel('Amplitude');

subplot(2, 2, 3);
plot(0:(Fs/length(t)):(Fs-Fs/length(t)), power_spectrum_original);
title('Power Spectrum (Original)');
xlabel('Frequency (Hz)');
ylabel('Power');

subplot(2, 2, 4);
plot(0:(Fs/length(t)):(Fs-Fs/length(t)), power_spectrum_reconstructed);
title('Power Spectrum (Reconstructed)');
xlabel('Frequency (Hz)');
ylabel('Power');

% Filter out desired frequencies for the reconstructed signal
filtered_reconstructed_signal = ifft(fft_signal .* (abs(fft_signal) > filter_freq));

% Display the filtered signal
figure;
subplot(2, 1, 1);
plot(t, real(filtered_reconstructed_signal));
title('Filtered Reconstructed Signal');
xlabel('Time (s)');
ylabel('Amplitude');

% Calculate the power spectrum of the filtered signal
power_spectrum_filtered = abs(fft(filtered_reconstructed_signal)).^2 / length(t);

subplot(2, 1, 2);
plot(0:(Fs/length(t)):(Fs-Fs/length(t)), power_spectrum_filtered);
title('Power Spectrum (Filtered Reconstructed)');
xlabel('Frequency (Hz)');
ylabel('Power');
