
% no fault, no noise
% output and faults
subplot(1,2,1)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,1),'b-', tout(:),Yout(1:tmax,2),'g-', ... % ch1: plant out, ch2: plant out
     tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--',...  % fault y1 , fault y2
     tout(:),Yout(1:tmax,5),'r--') % fault u1
legend('Plant Output 1','Plant Output 2','Fault Output 1','Fault Output 2','Fault Input')
title('No Fault - Outputs')
% 
subplot(1,2,2)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--', tout(:), Yout(1:tmax,5),'r--', ... % faults
     tout(:),Rout(1:tmax,1),'g-', tout(:),Rout(1:tmax,2),'m-',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'k-'); % residual 3
legend('Fault Output 1','Fault Output 2','Fault Input','Residual 1','Residual 2','Residual 3')
title('No Fault - Residuals')

%%
% input fault, no noise
% output and faults
subplot(1,2,1)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,1),'b-', tout(:),Yout(1:tmax,2),'g-', ... % ch1: plant out, ch2: plant out
     tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--',...  % fault y1 , fault y2
     tout(:),Yout(1:tmax,5),'r--') % fault u1
legend('Plant Output 1','Plant Output 2','Fault Output 1','Fault Output 2','Fault Input')
title('Input Fault - Outputs')
% 
subplot(1,2,2)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--', tout(:), Yout(1:tmax,5),'r--', ... % faults
     tout(:),Rout(1:tmax,1),'g-', tout(:),Rout(1:tmax,2),'m-',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'k-'); % residual 3
legend('Fault Output 1','Fault Output 2','Fault Input','Residual 1','Residual 2','Residual 3')
title('Input Fault - Residuals')

%%
% output Fy1 fault, no noise
% output and faults
subplot(1,2,1)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,1),'b-', tout(:),Yout(1:tmax,2),'g-', ... % ch1: plant out, ch2: plant out
     tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--',...  % fault y1 , fault y2
     tout(:),Yout(1:tmax,5),'r--') % fault u1
legend('Plant Output 1','Plant Output 2','Fault Output 1','Fault Output 2','Fault Input')
title('Output Fy1 Fault - Outputs')
% 
subplot(1,2,2)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--', tout(:), Yout(1:tmax,5),'r--', ... % faults
     tout(:),Rout(1:tmax,1),'g-', tout(:),Rout(1:tmax,2),'m-',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'k-'); % residual 3
legend('Fault Output 1','Fault Output 2','Fault Input','Residual 1','Residual 2','Residual 3')
title('Output Fy1 Fault - Residuals')

%%
% output Fy2 fault, no noise
% output and faults
subplot(1,2,1)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,1),'b-', tout(:),Yout(1:tmax,2),'g-', ... % ch1: plant out, ch2: plant out
     tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--',...  % fault y1 , fault y2
     tout(:),Yout(1:tmax,5),'r--') % fault u1
legend('Plant Output 1','Plant Output 2','Fault Output 1','Fault Output 2','Fault Input')
title('Output Fy2 Fault - Outputs')
% 
subplot(1,2,2)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--', tout(:), Yout(1:tmax,5),'r--', ... % faults
     tout(:),Rout(1:tmax,1),'g-', tout(:),Rout(1:tmax,2),'m-',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'k-'); % residual 3
legend('Fault Output 1','Fault Output 2','Fault Input','Residual 1','Residual 2','Residual 3')
title('Output Fy2 Fault - Residuals')

%%
%%









% no fault, with output noise
% output and faults
subplot(1,2,1)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,1),'b-', tout(:),Yout(1:tmax,2),'g-', ... % ch1: plant out, ch2: plant out
     tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--',...  % fault y1 , fault y2
     tout(:),Yout(1:tmax,5),'r--') % fault u1
legend('Plant Output 1','Plant Output 2','Fault Output 1','Fault Output 2','Fault Input')
title('No Fault, with 0.01 Output Noise ON1 and ON2 - Outputs')
% 
subplot(1,2,2)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--', tout(:), Yout(1:tmax,5),'r--', ... % faults
     tout(:),Rout(1:tmax,1),'g-', tout(:),Rout(1:tmax,2),'m-',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'k-'); % residual 3
legend('Fault Output 1','Fault Output 2','Fault Input','Residual 1','Residual 2','Residual 3')
title('No Fault, with 0.01 Output Noise ON1 and ON2 - Residuals')

%%
%%
% input fault, 0.1 output noise noise
% output and faults
subplot(1,2,1)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,1),'b-', tout(:),Yout(1:tmax,2),'g-', ... % ch1: plant out, ch2: plant out
     tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--',...  % fault y1 , fault y2
     tout(:),Yout(1:tmax,5),'r--') % fault u1
legend('Plant Output 1','Plant Output 2','Fault Output 1','Fault Output 2','Fault Input')
title('Input Fault, with 0.1 Output Noise ON1 and ON2 - Outputs')
% 
subplot(1,2,2)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--', tout(:), Yout(1:tmax,5),'r--', ... % faults
     tout(:),Rout(1:tmax,1),'g-', tout(:),Rout(1:tmax,2),'m-',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'k-'); % residual 3
legend('Fault Output 1','Fault Output 2','Fault Input','Residual 1','Residual 2','Residual 3')
title('Input Fault, with 0.1 Output Noise ON1 and ON2 - Residuals')

%%
% output Fy1 fault, 0.1 output noise noise
% output and faults
subplot(1,2,1)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,1),'b-', tout(:),Yout(1:tmax,2),'g-', ... % ch1: plant out, ch2: plant out
     tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--',...  % fault y1 , fault y2
     tout(:),Yout(1:tmax,5),'r--') % fault u1
legend('Plant Output 1','Plant Output 2','Fault Output 1','Fault Output 2','Fault Input')
title('Output Fy1 Fault, with 0.1 Output Noise ON1 and ON2 - Outputs')
% 
subplot(1,2,2)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--', tout(:), Yout(1:tmax,5),'r--', ... % faults
     tout(:),Rout(1:tmax,1),'g-', tout(:),Rout(1:tmax,2),'m-',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'k-'); % residual 3
legend('Fault Output 1','Fault Output 2','Fault Input','Residual 1','Residual 2','Residual 3')
title('Output Fy1 Fault, with 0.1 Output Noise ON1 and ON2 - Residuals')








%%
%%

% input fault, 0.05 input noise noise
% output and faults
subplot(1,2,1)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,1),'b-', tout(:),Yout(1:tmax,2),'g-', ... % ch1: plant out, ch2: plant out
     tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--',...  % fault y1 , fault y2
     tout(:),Yout(1:tmax,5),'r--') % fault u1
legend('Plant Output 1','Plant Output 2','Fault Output 1','Fault Output 2','Fault Input')
title('Input Fault, with 0.05 Input Noise - Outputs')
% 
subplot(1,2,2)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--', tout(:), Yout(1:tmax,5),'r--', ... % faults
     tout(:),Rout(1:tmax,1),'g-', tout(:),Rout(1:tmax,2),'m-',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'k-'); % residual 3
legend('Fault Output 1','Fault Output 2','Fault Input','Residual 1','Residual 2','Residual 3')
title('Input Fault, with 0.05 Input Noise - Residuals')

%%
% input fault, 0.05 input noise noise
% output and faults
subplot(1,2,1)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,1),'b-', tout(:),Yout(1:tmax,2),'g-', ... % ch1: plant out, ch2: plant out
     tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--',...  % fault y1 , fault y2
     tout(:),Yout(1:tmax,5),'r--') % fault u1
legend('Plant Output 1','Plant Output 2','Fault Output 1','Fault Output 2','Fault Input')
title('Output Fault, with 0.01 Input Noise - Outputs')
% 
subplot(1,2,2)
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,3),'r--', tout(:),Yout(1:tmax,4),'r--', tout(:), Yout(1:tmax,5),'r--', ... % faults
     tout(:),Rout(1:tmax,1),'g-', tout(:),Rout(1:tmax,2),'m-',...  % residual y1 , residual y2
     tout(:),Rout(1:tmax,3),'k-'); % residual 3
legend('Fault Output 1','Fault Output 2','Fault Input','Residual 1','Residual 2','Residual 3')
title('Output Fault, with 0.01 Input Noise - Residuals')