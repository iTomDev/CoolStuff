

%% 
% Loop Shaping
% Inverted pendulum, h8 loop shaped, old style Matlab code.
% Based on Design_CartPend_LSDP_V2.1 
%
% Works ish, but cant use uncertain plants e.g uss. Robustness can only be
% analysed in the sense of gamma and the stability margins, which whilst
% sufficient for robustness isn't exactly "complete".
%
% "The closed-loop system CL represents the transfer matrix from the 
% reference and disturbance to the feedback error and output of W1." from
% matlab help. 
% Our W1 has 1 in 1 out, so is probably control input from K.
% The other 2 presumably are feedback error of theta and x. Just a guess!
% Still, we have 3 inputs, since there is no reference for theta, perhaps
% that is ref x, dist x, and dist theta? Again, just a guess!
% see MAtlab mu docs page 431 for info on adding a reference. I think it
% only works with the old version of ncfsyn?
%
% 

%%
% B1. Plant
clc
clear

% parameters (previous working)
M = 0.5;
m = 0.2;
l = 0.3;
b = 0.1;
I = 0.006;
g = 9.8;   % gravity

% parameters (changed to match Design_CartPend_3)
M = 0.2;
m = 0.2;
l = 0.3;
b = 0.001;    % Cart friction coefficient [N/m/sec]
I = 0.006;
g = 9.8;   % gravity

% state equations
p = I*(M+m)+M*m*l^2; % denominator for the A and B matrices
A = [0       1              0         0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p  0;
     0       0              0         1;
     0   -(m*l*b)/p    m*g*l*(M+m)/p  0];
B = [0;
     (I+m*l^2)/p;
     0;
     m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];

% states = {'x' 'x_dot' 'phi' 'phi_dot'};
% inputs = {'u'}; 
% outputs = {'x' 'phi'};
G = ss(A,B,C,D);
G

%%

% bode analysis
bode(G)        % bode of plant
bode(G(1,1))   % bode of input 1 to output 1. 
bode(G(2,1))   % bode of input 1 to output 2

sigma(G)

% ch1 phase crossover ~20rad/sec, but is nearly there at about 4 rad/sec

% root locus analysis
rlocus(G(1,1))
rlocus(G(2,1))

%%
% B3. Weights for loop shaped plant and controller synthesis
% using nominal model of plant, no real or complex uncertainties

W1 = tf([1], [0.01 .002]);
W2 = [1 0; 0 1];

% response of weights
bode(W1)

% shaped plant
Gs = W2*G*W1;
sigma(Gs, {1E-3 1E3});
% plot plant and shaped plant
% Red is original plant, green is shaped plant
% this has higher gain is low w, lower at high w 
sigma(G, 'r--', Gs, 'g--',  {1E-3 1E3});
grid

[Klsh, CL, GAM] = ncfsyn(G, W1, W2);
Klsh
GAM % aim for 1<GAM<3
K = W1*Klsh*W2;

size(K);
size(G);
size(CL)

% compare starp closed loop with matlab produced version
clp = feedback(G, -K);
clp;
CL;

% get dimensions of G, K etc
size(CL);
size(clp);
size(K)

%%
% B4. Analyse closed loop plant

bode(CL)

% gives garbage
L=G*Klsh;
I=eye(size(L));
S=feedback(I,L); % S=inv(I+L);
T=I-S;
step(T);title('\alpha and \theta command step responses');

% Plot S and T over omega alongside their "ideal" lines
% the red solid line should match the red dashed line ideally, same goes
% for the black lines
% small S at low freq, small t at high freq
%L = W1*Klsh*W2*G;
L = K*Gs;
S = inv(1+L);
T = 1-S;
sigma(S, 'k', GAM/W1, 'k-.', T, 'r', GAM*G/W2, 'r-.')
legend('S=1/(1+L)', 'GAM*W1', 'T=L(1+L)', 'GAM*P/W2',2)

% plot GK and check it is within 20log10(GAM) [dB] of ideal loop shape
% K = w1*klsh*w2, so GK is really W2*G*W1*Klsh
sigma(G*K,'b',G*W1,'g-.',G*W1*GAM,'k-.',G*W1/GAM,'k-.')


% "The closed-loop system CL represents the transfer matrix from the 
% reference and disturbance to the feedback error and output of W1.
% 1-> 1,2: disturbance rejection, both channels
% 2-> 1,2: reference tracking, both channels
% 3-> 1,2: disturbancerejection on both, slight SSE on xart pos
% 1-> 3: disturbance rejection
% 2-> 3: disturbance rejection
% 3-> 3: reference tracking
% What we do know is that 3 is the input, and (1,2) are the outputs.
size(CL)
% simulation
% need a vector of inputs in this case, u3 is the original input, y1,y2 are
% the original outputs
t = 0:0.1:6;
sizet = size(t);
r = zeros(3,sizet(2));        % 3*t array
r(3,:) = 0.2*ones(size(t));  % step into channel 3
r
[y,t,x] = lsim(CL,r,t);
[AX,H1,H1] = plotyy(t, y(:,1), t, y(:,2), 'plot');
set(get(AX(1),'YLabel'), 'String','Cart position [m]')
set(get(AX(2),'YLabel'), 'String','Pendulum angle [radians]')
title('step response with loop shaped h-infinity controller')

% plot singular values of the closed loop
L = K*G;
sigma(L, 'b-', inv(W1), 'r-');

% from matlab docs
sigma(G,'b-.',G*K,'g',W2*G*W1/GAM,'r:',W2*G*W1*GAM,'r:',{.1,30})

T=feedback(G*K,eye(2));
sigma(T,ss(GAM),'k:',{.1,30});
grid

% nyquist(CL)
nyquist(CL(1,3))
nyquist(CL(2,3))

% pzplot
pzplot(CL(1,3))
pzplot(CL(2,3))

% root locus
rlocus(CL(1,3))
rlocus(CL(2,3))


%%
%%
% mu analysis of lsdp controller

% compute mu bounds as there is strutured uncertainty
% problem 1: matin dimensions not compatible
% solution 1: use clp2_g = frsp(clp2, omega); with all rows and columns,
% dont select any
% problem 2: lower mu bound is higher than upper mu bound
%clp_lsdp
omega = logspace(-2,4,100); % omega = logspace(-5,5,100);
clp2 = CL;%starp(wolp_ic, K);
clp2_g = frsp(clp2, omega);  
minfo(clp2_g) % 100 pts     8 rows     7 cols
blockrsR = [-1 1; -1 1; -1 1; -1 1; -1 1; -1 1]; % 6 real uncertainties
rob_stab = sel(clp2_g, [1:6], [1:6]); % uncertaint IOs 1-6
pdim = ynum(rob_stab);                  % pdim = 1
fixl = [eye(pdim); 0.01*eye(pdim)]; 
fixr = fixl';
blkrs = [blockrsR; abs(blockrsR)];
clp_mix = mmult(fixl, rob_stab, fixr);
minfo(blkrs) % 12 rows  2 cols
minfo(clp_mix)  %   100 pts     2 rows     2 col
[rbnds,rowd,sens,rowp,rowg] = mu(clp_mix, blkrs);  
vplot('liv,lm', sel(rbnds,1,1),'r-'...   % dashed - probably the upper
                ,sel(rbnds,1,2),'g--'...   % solid - probably the lower
                ,vnorm(rob_stab),'c-.')   % h8 norm bound 
title('mu Analysis: Robust Stability');
xlabel('Frequency [rad/sec]')
ylabel('mu');
