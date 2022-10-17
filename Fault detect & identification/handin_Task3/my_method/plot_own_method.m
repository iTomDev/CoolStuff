
% no fault, with noise
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,1),'b-', tout(:),Yout(1:tmax,2),'r-', ... % ch1: plant out, ch2: plant out
     tout(:),Rout(1:tmax,1),'g-', tout(:),Rout(1:tmax,2),'m-',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'k--'); % residual u1
legend('Output 1 Plant','Output 2 Plant','Output 1 Residual','Output 2 Residual','Output 3 Residual')
title('Input Fault')

% Input Fault with no noise
tmax = size(tout,1);
plot(...
     tout(:),Rout(1:tmax,1),'r--', tout(:),Rout(1:tmax,2),'g--',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'b--'); % residual u1
legend('Output 1 Residual','Output 2 Residual','Output 3 Residual')
title('Input Fault, no noise')

% Output Fault with no noise y1
tmax = size(tout,1);
plot(...
     tout(:),Rout(1:tmax,1),'r--', tout(:),Rout(1:tmax,2),'g--',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'b--'); % residual u1
legend('Output 1 Plant','Output 2 Plant','Output 1 Residual','Output 2 Residual','Output 3 Residual')
title('Output Fault Fy1, no noise')

% Output Fault with no noise y2
tmax = size(tout,1);
plot(...
     tout(:),Rout(1:tmax,1),'r--', tout(:),Rout(1:tmax,2),'g--',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'b--'); % residual u1
legend('Output 1 Plant','Output 2 Plant','Output 1 Residual','Output 2 Residual','Output 3 Residual')
title('Output Fault Fy2, no noise')

% Input Fault with noise
tmax = size(tout,1);
plot(...
     tout(:),Rout(1:tmax,1),'r--', tout(:),Rout(1:tmax,2),'g--',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'b--'); % residual u1
legend('Output 1 Residual','Output 2 Residual','Output 3 Residual')
title('Input Fault, with noise')

% Output Fault with noise y1
tmax = size(tout,1);
plot(...
     tout(:),Rout(1:tmax,1),'r--', tout(:),Rout(1:tmax,2),'g--',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'b--'); % residual u1
legend('Output 1 Residual','Output 2 Residual','Output 3 Residual')
title('Output Fault Fy1, with noise')

%%

% Output Fault with negative fault
tmax = size(tout,1);
plot(...
     tout(:),Rout(1:tmax,1),'r--', tout(:),Rout(1:tmax,2),'g--',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'b--'); % residual u1
legend('Output 1 Residual','Output 2 Residual','Output 3 Residual')
title('Output Negative Fault Fy1')

% Output Fault Fy1, 1Hz Sine Wave
tmax = size(tout,1);
plot(...
     tout(:),Rout(1:tmax,1),'r--', tout(:),Rout(1:tmax,2),'g--',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'b--'); % residual u1
legend('Output 1 Residual','Output 2 Residual','Output 3 Residual')
title('Output Fault Fy1, 1Hz Sine Wave')

% Output Fault Fy1, 50Hz Sine Wave
tmax = size(tout,1);
plot(...
     tout(:),Rout(1:tmax,1),'r--', tout(:),Rout(1:tmax,2),'g--',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'b--'); % residual u1
legend('Output 1 Residual','Output 2 Residual','Output 3 Residual')
title('Output Fault Fy1, 50Hz Sine Wave')

% Input Fault, 1Hz Sine Wave
tmax = size(tout,1);
plot(...
     tout(:),Rout(1:tmax,1),'r--', tout(:),Rout(1:tmax,2),'g--',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'b--'); % residual u1
legend('Output 1 Residual','Output 2 Residual','Output 3 Residual')
title('Input Fault Fy1, 1Hz Sine Wave')