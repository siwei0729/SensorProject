function fre = getCutOffFre(x,Fs)

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
   %plot(f,abs(X)/N);
   %disp(abs(X(5600:6000))/N)
   %disp(f)
   

   order = 1;
   [b, a] = butter(order, 0.1, 'low'); % 1/5 of its f
   X = filtfilt(b, a, X);
   
   X = abs(X)/N*50;
   figure;
   plot(f,X);
   xlabel('Frequency (in hertz)');
   title('Spectrum after low pass');
   axis([-5 5 0 10]);
    
   interval = 2;
   lastTan = 0;
   for i=int16(length(X)/2+1):1:length(X)
       
       if i+interval < length(X)
           VD = X(i) - X(i+interval-1);
           HD = abs(f(i) - f(i+interval-1));
%            disp(VD)
%            disp(HD)
        if lastTan == 0
            lastTan = VD/HD;
        end
        currentTan = VD/HD;
        disp(currentTan)
        if currentTan < 0
             disp(i)
             fre = f(i);
             break;
        end
        
        if abs(currentTan)<abs(lastTan) && currentTan>0 
           if currentTan <=3 % times 50 is to convert to X and Y to same scale
               disp(i)
               fre = f(i);
               break;
           end
        end
        lastTan = currentTan;
      end
   end
   
   
   
%    for i=int16(length(X)/2+1):20:length(X)
%        sum=0;
%        for j=i:1:i+19
%             sum = sum +abs(X(j))/N;
%        end
%        
%        stop = true;
%        for j=0:1:2
%            sumTemp =0;
%            if i+j*20+19 < length(X)
%                 for k=i+j*20:1:i+j*20+19
%                     sumTemp = sumTemp +abs(X(k))/N;
%                 end
%                 disp(sumTemp)
%                 if sum - sumTemp > 0.0001
%                     stop = false;
%                 end
%            end
%        end
%        if stop
%            fre = f(i);
%            disp('i:')
%            disp(i)
%            disp(abs(X(k))/N)
%            break;
%        end
%        %disp(sumTemp)
%    end
   
end