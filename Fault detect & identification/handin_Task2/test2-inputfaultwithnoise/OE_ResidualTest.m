
%%
% Input Fault with Additive Input and output Noise
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,1),'b-', tout(:),Yout(1:tmax,2),'r-', ... % ch1: plant out, residual
     tout(:),Yout(1:tmax,3),'g-', tout(:),Yout(1:tmax,4),'m-',...  % ch2: plant out, residual
     tout(:),ones(1,tmax)*30,'k--', tout(:),ones(1,tmax)*-30,'k--'); % threshold line, max/min
legend('Output 1 Plant','Output 1 Residual','Output 2 Plant','Output 2 Residual')
title('Input Fault with Additive Input and Output Noise')

%%
% Input Fault with Additive Input Noise
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,1),'b-', tout(:),Yout(1:tmax,2),'r-', ... % ch1: plant out, residual
     tout(:),Yout(1:tmax,3),'g-', tout(:),Yout(1:tmax,4),'m-',...  % ch2: plant out, residual
     tout(:),ones(1,tmax)*30,'k--', tout(:),ones(1,tmax)*-30,'k--'); % threshold line, max/min
legend('Output 1 Plant','Output 1 Residual','Output 2 Plant','Output 2 Residual')
title('Input Fault with Additive Input Noise')


%%
% Input Fault with Additive Output Noise
tmax = size(tout,1);
plot(tout(:),Yout(1:tmax,1),'b-', tout(:),Yout(1:tmax,2),'r-', ... % ch1: plant out, residual
     tout(:),Yout(1:tmax,3),'g-', tout(:),Yout(1:tmax,4),'m-',...  % ch2: plant out, residual
     tout(:),ones(1,tmax)*9,'k--', tout(:),ones(1,tmax)*-9,'k--'); % threshold line, max/min
legend('Output 1 Plant','Output 1 Residual','Output 2 Plant','Output 2 Residual')
title('Input Fault with Additive Output Noise')