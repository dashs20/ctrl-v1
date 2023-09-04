clear;
clc;

Fs = 2000; % Sampling rate in Hz
T = 3;     % Total duration of the signal in seconds
t = 0:1/Fs:T-1/Fs; % Time vector

% Create a sine wave with various frequencies
f = [10,100,800]; % frequencies of signal
x = zeros(1,length(t));
for i = f
    x = x + sin(2*pi*i*t);
end

% Compute the FFT
X = fft(x);

% Calculate the corresponding frequencies
N = length(x);
frequencies = (0:N/2-1) * (Fs/N); % Positive frequencies
frequencies = [frequencies, -fliplr(frequencies(2:end))]; % Include negative frequencies

% Modify X to remove frequencies you don't like
cutoff_frequency = 10; % Frequencies above this value will be removed
X(abs(frequencies) > cutoff_frequency) = 0;

% Reconstruct the signal by taking the inverse FFT
reconstructed_signal = ifft(X);

% Plot the original and reconstructed signals
subplot(2,1,1);
plot(t, x);
title('Original Signal');

subplot(2,1,2);
plot(t, real(reconstructed_signal)); % Use real() to handle potential small imaginary parts
title('Reconstructed Signal');

xlabel('Time (s)');
