function spectrum(x,Fs,titleT)

%% Time specifications:                     % samples per second
   %dt = 1/Fs;                     % seconds per sample
   %StopTime = 1;                  % seconds
   %t = (0:dt:StopTime-dt)';
   N = size(x,1);

   %% Fourier Transform:
   X = fftshift(fft(x));
   %% Frequency specifications:
   dF = Fs/N;                      % hertz
   f = -Fs/2:dF:Fs/2-dF;           % hertz
   %% Plot the spectrum:
   figure;
   plot(f,abs(X)/N);
   xlabel('Frequency (in hertz)');
   title(titleT);
   axis([-5 5 0 0.18]);
   %disp(abs(X(5600:6000))/N)
   %disp(f)
end