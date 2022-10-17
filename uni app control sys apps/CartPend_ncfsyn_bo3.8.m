
%%
% Standard hinfsyn method

% A1. Uncertain plant

clc
clear

% parameters
M = 0.2;
m = 0.2;
l = 0.3;
%l = ureal('l', 0.1, 'percentage', 10);
 b = ureal('b', 0.001, 'percentage', 10);    % Cart friction coefficient [N/m/sec]
% I = ureal('I', 0.006, 'percentage', 10);    % Mass moment of inertia of pendulum [Kg.m^2]
%b = 0.1;
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

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'control'}; % was in
outputs = {'alpha' 'theta'}; % was x, phi

G = uss(A, B, C, D, 'inputname', inputs, 'outputname', outputs);

systemnames = ' G ';
inputvar = '[ref; dist; control]';
outputvar = '[G(1)-dist; G(2)-dist; ref - G(1) - dist]'; % x+dist; phi+dist; ref-x-dist
input_to_G = '[control]';
sysoutname = 'simol_ic';   %sim_ic
cleanupsysic = 'yes';
sysic;
simol_ic

[solA solB solC solD] = ssdata(simol_ic);
simolic_inputs = {'ref' 'dist' 'control'};
simolic_outputs = {'altalpha' 'theta' 'alpha'};
simol_ic2 = ss(solA, solB, solC, solD, 'inputname', simolic_inputs, 'outputname', simolic_outputs);

Gnom = G.nominal;

%%
% A2. Open loop analysis

pole(G.nominal)
zero(G.nominal)
% pole/zero plot
pzplot(G.nominal)
pzplot(G.nominal(1,1))
pzplot(G.nominal(2,1))

% time resp - about 1.04s for pend to fall over
step(G.nominal, 1.04)

% nyquist of nominal plant
nyquist(G.nominal(1,1))
nyquist(G.nominal(2,1))
% root locus
rlocus(G.nominal(1,1))
rlocus(G.nominal(2,1))

% bode
bode(G)
grid

% Plot maximum gain as a function of frequency
sigma(G, 'g', {.0001, 10000})
sigma(G)
title('Singular value plot for plant')

% nominal plant
Gnom = G.nominal;

%%
% A3. Design weights
s = tf('s');

% control weight
Wu = tf([1],[1 0.001]); % 2.0000e+004
Wu = 0.5;
%bode(Wu, 'g-', Gnom, 'r-');

% performance weight
Wpa = tf([30 20], [20 1]);  % 
Wpb = tf([20 -0.1], [800 1]);
Wp= [Wpa 0; 0 Wpb];
bode(Wp)
grid

%%

%open loop model
systemnames = 'G Wu Wp';
inputvar = '[ref; dist{2}; control]';
outputvar = '[-Wu; Wp; ref-G(1)-dist(1); G(2)+dist(2)]'; %Wu; Wp {2}; pend angle; car position
input_to_Wu = '[control]';
input_to_G = '[control]';
input_to_Wp = '[G(1)+dist(1); G(2)+dist(2)]'; % x, theta
sysoutname = 'wplant';
cleanupsysic = 'yes';
sysic;
wplant

% obtain system matrix for hinfsyn
[M Delta] = lftdata(wplant);
[Ma Mb Mc Md] = ssdata(M);
wplantinputs = {'dist1' 'dist2' 'control'};
wplantoutputs = {'Wu' 'Wp1' 'Wp2' 'thetadist' 'alphadist'};
wplant1 = ss(Ma, Mb, Mc, Md); % 'inputname', wplantinputs, 'outputname', wplantoutputs);
wplant1

% create a controller
% [k,g,gfin] = hinfsyn(p,nmeas,ncon,gmin,gmax,tol,ricmethd,epr,epp
nmeas = 2; % row dim of C2
ncont = 1; % column dim of B2
%[Khinf stuff more] = hinfsyn(wplant1, nmeas, ncont, 0.1, 10, .001);
[Khinf CL GAM] = hinfsyn(wplant1, nmeas, ncont);
Khinf
CL
GAM

%%
% A4. Controller analysis
omega = logspace(-1,2);
sigma(CL, omega)

% step response of CL up until 1 second
% IMO in(3) is the ref input, out(3,4) are x and phi
step(CL, 1.04)

% 
sigma(CL, 'b--')

% nominal performance
sizeG = size(G);
S = inv((eye(sizeG(1)))+G*Khinf);
sigma(S, 'b--'); 

% S vs channel 1 inv performance weight
sigma(S, 'b--', inv(Wpa), 'r'); 

% S vs channel 2 inv performance weight
sigma(S, 'b--', inv(Wpb), 'r'); 

% T
T = G*Khinf*S;
sigma(T)

% KS
KS = Khinf*S;
sigma(KS)

% systemnames = 'G Wu Wp';
% inputvar = '[dist; control]';
% outputvar = '[-Wu; Wp; G(2)-dist; -G(1)]'; %Wu; Wp {2}; pend angle; car position
% input_to_Wu = '[control]';
% input_to_G = '[control]';
% input_to_Wp = '[G(1); G(2)-dist]'; % x, theta
% sysoutname = 'wplant';
% cleanupsysic = 'yes';
% sysic;
% wplant

%%

% open loop simulation plant
systemnames = ' G ';
inputvar = '[ref; dist{2}; control]';
outputvar = '[ref-G(1)+dist(1); G(2)+dist(2)]'; % 
input_to_G = '[control]';
sysoutname = 'simol_ic';   %sim_ic
cleanupsysic = 'yes';
sysic;
simol_ic

% using loopsens, not working so well though!
loops = loopsens(simol_ic, Khinf);
sigma(loops.So)
step(loops.So, 1.04)

% S is high at low freq, poor disturbance attenuation
loops = loopsens(Gnom, Khinf);
sigma(loops.So)

% alternate calculation of disturbance rejection
GK = ssdata(Gnom*Khinf);
step(inv(eye(10)+GK))

% convert old to use starp
[pa pb pc pd] = ssdata(simol_ic.nominal);
simol_ic2 = pck(pa, pb, pc, pd);
size(simol_ic2) %9 out, 8
[ka kb kc kd] = ssdata(Khinf);
Khinf2 = pck(ka, kb, kc, kd);
size(Khinf2) %8 out, 9
L2 = starp(simol_ic2,Khinf2 );
size(L2) % 13 out, 13

% convert modern for lsim
[acl bcl ccl dcl] = unpck(L2);
L3 = ss(acl,bcl,-ccl,-dcl);
size(L3) % 2 outputs, 2 inputs, and 10 state

% simulation - reference tracking (cart position is key here)
t = 0:0.1:90;
sizet = size(t);
r = zeros(3,sizet(2));        % 3*t array
r(3,:) = 1*ones(size(t));  % step into channel 1
r;
[y,t,x] = lsim(L3,r,t);
[AX,H1,H1] = plotyy(t, y(:,1), t, y(:,2), 'plot');
set(get(AX(1),'YLabel'), 'String','Cart position [m]')
set(get(AX(2),'YLabel'), 'String','Pendulum angle [radians]')
title('step response with loop shaped h-infinity controller')

% CL and the starp result are the saem, only difference is CL is using
% positive feedback!

%%

CL
% the size changes with the number of uncertain params
% number of non-uncertain IOs in CL depends on wolp_ic
% State-space model with 5 outputs, 3 inputs, and 18 states
%      1   2,3   4    5
% out: Wu; Wp; G(1); G(2) = Wu; Wp; x; theta
% in: dist{1,2}; control
size(CL) 

% simulation - disturbance rejection (pendulum angle is key here)
t = 0:0.1:90;
sizet = size(t);
r = zeros(3,sizet(2));        % 3*t array
r(1,:) = .1*ones(size(t));  % step into channel 1
r;
[y,t,x] = lsim(CL,r,t);
[AX,H1,H1] = plotyy(t, y(:,2), t, y(:,3), 'plot');
%set(get(AX(1),'YLabel'), 'String','Cart position [m]')
set(get(AX(2),'YLabel'), 'String','Pendulum angle [radians]')
title('step response with loop shaped h-infinity controller')

% simulation - reference tracking (cart position is key here)
t = 0:0.1:90;
sizet = size(t);
r = zeros(3,sizet(2));        % 3*t array
r(3,:) = 1*ones(size(t));  % step into channel 1
r;
[y,t,x] = lsim(CL,r,t);
[AX,H1,H1] = plotyy(t, y(:,3), t, y(:,4), 'plot');
set(get(AX(1),'YLabel'), 'String','Cart position [m]')
%set(get(AX(2),'YLabel'), 'String','Pendulum angle [radians]')
title('step response with loop shaped h-infinity controller')

%%

% check energy used by controller
Khinf;

sigma(Khinf)
sigma(CL)

%%
% new style mu analysis code

% Howto
% 1. use lft to join simulation model with controller
% CL3 = lft(simol_ic, Khinf) 
% 2. use lftdata to extract the model and block structure. You want 1 and 3
% [M,DELTA,BLKSTRUCT] = lftdata(CL3);
% 3. 
% bounds = mussv(M,BlockStructure)

% compute mu bounds
CL3 = lft(simol_ic, Khinf);
[M,DELTA,BLKSTRUCT] = lftdata(CL3);
M          % uncertainty interconnection model. has controller so its really P 
DELTA      % normalised uncertainties
BLKSTRUCT  % uncertainty block structure
omega = logspace(-2,4,100);
szDelta = size(DELTA); % size of delta corresponds to # of uncerts
M2 = M(1:szDelta(2), 1:szDelta(1)); % select uncertainty channels
M2_g = frd(M2, omega); % get frequency response
mubounds = mussv(M2_g, BLKSTRUCT, 's') % compute bounds

% plot mu bounds
LinMagopt = bodeoptions;
LinMagopt.PhaseVisible = 'off'; 
LinMagopt.XLim = [1e-2, 1e4];
LinMagopt.MagUnits = 'abs';
bodeplot(mubounds(1,1), mubounds(1,2), LinMagopt);
xlabel('Frequency [rad/sec]');
ylabel('Mu Upper/Lower Bounds');
title('Mu Analysis: Robust Stability Margins');
