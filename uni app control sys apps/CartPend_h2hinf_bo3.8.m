
%%
% Mixed H2/H8 syntheis controller for inverted Cart Pendulum system
% Simple pole placement constraints (must be in LHP)
%
% Based on Design_CartPend_3.8
% Thomas Pile Sept 2014

% Uncertain plant
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
outputs = {'alpha' 'theta'}; % was x, phi. 

G = uss(A, B, C, D, 'inputname', inputs, 'outputname', outputs);

Gnom = G.nominal; % need Gnom for sysics

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

%%
% define weights
Wu_H2 =  tf([100],[1000 100]);
Wu_H8 = tf([0.5],[0.1 1]);
Wp_S = tf([500 10], [10 500]); 
%Wp_S = [Wp_Sa 0; 0 Wp_Sa];

%%
% check weights and compare to nominal plants
bode(Wp_S)
bode(Wu_H8)
bode(Wu_H2)
bode(Gnom(1,1), Gnom(2,1),Wp_S, Wu_H8, Wu_H2)

% use channel 2 as its the primary objective output
sigma(Gnom(2,1), 'k', Wp_S, 'r', Wu_H8, 'g', Wu_H2, 'b')

%%
% form open loop objective plant

% adapted from Gu - needs work
% sign changes on inputs and outputs just change corresponding out/in, not
% a big deal, fix when things work well
% H2 and H8 weights are on u, S on G(2)
systemnames = 'Gnom Wp_S Wu_H8 Wu_H2';
inputvar = 'dist;ref;u';
outputvar = '[Wp_S;Wu_H8;Wu_H2;ref+Gnom(1);Gnom(2)+dist]'; 
input_to_Gnom = '[u]';
input_to_Wu_H8 = '[u]';
input_to_Wu_H2 = '[u]';
input_to_Wp_S = '[Gnom(2)+dist]';
sysoutname = 'P';
cleanupsysic = 'yes';
sysic;
inputs = {'dist' 'ref' 'control'};
P1 = ltisys(P.a,P.b,P.c,P.d);
minfo(P1)

%%
% Syntheis procedure

% LMI Objectives
% [0 0 0 0] pole placement only
% [0 0 1 0] h8 optimal design
% [0 0 0 1] h2 optimal design
% [g 0 0 1] min the 2 norm of the 2norm TF subject to inf norm of inf norm TF being < g
% [0 0 a b] babalce between h8 and h2 weighted by constants

% mixed h2/hinf, right ballpark
z2 = 1; nmeas = 2; ncont = 1; % z2 = 1; nmeas = 2; ncont = 1;
r = [z2 nmeas ncont];
obj = [0 0 1 0]; % min hinf
%obj = [0 0 0 1]; % min h2
obj = [0 500 1 0]; % min hinf s/t h2<500 ?
obj = [100 0 0 1]; %h2 s/t hinf < 100

%   region = [ -95.8851 + 2.0000i 0        0.4794 -0.8776;         
%             0                   -95.8851 0.8776 0.4794];
[Gnomopt,h2opt,Kmix] = hinfmix(P1,r,obj);
Kmix

%%
% closed loop and time responses using connect
kinputs = {'alpha' 'theta'}; 
koutputs = {'control'}; 
[Kmixa,Kmixb,Kmixc,Kmixd] = ltiss(Kmix);
Kmix2 = ss(Kmixa,Kmixb,Kmixc,Kmixd, 'inputname', kinputs, 'outputname', koutputs);
pinputs = {'ref' 'dist' 'control'};  
poutputs = {'Wp_S' 'Wu_H8' 'Wu_H2' 'alpha' 'theta'};
[Pmixa,Pmixb,Pmixc,Pmixd] = ltiss(P1);
Pmix2 = ss(Pmixa,Pmixb,Pmixc,Pmixd, 'inputname', pinputs, 'outputname', poutputs);

% form closed loop with connect
% these inputs and outputs are HUGE. Radians and meters!
subplot(2,2,1);
CLmixA = connect(Kmix2,Pmix2,'dist','alpha');
step(CLmixA)
subplot(2,2,2);
CLmixB = connect(Kmix2,Pmix2,'ref','alpha');
step(CLmixB)
subplot(2,2,3);
CLmixC = connect(Kmix2,Pmix2,'ref','theta');
step(CLmixC)
subplot(2,2,4);
CLmixD = connect(Kmix2,Pmix2,'dist','theta');
step(CLmixD)

% ref to cart position
lsimdt = 0.001;
lsimfinal = 10;
lsimt = 0:lsimdt:lsimfinal;
sizet = size(lsimt);
lsimu = zeros(1,sizet(2));
lsimu(1,1:1) = 1;
lsim(CLmixB, lsimu', lsimt); 

% dist to theta
lsimdt = 0.01;
lsimfinal = 10;
lsimt = 0:lsimdt:lsimfinal;
sizet = size(lsimt);
lsimu = zeros(1,sizet(2));
lsimu(1,1:10) = 0.1;
lsim(CLmixD, lsimu', lsimt); 

%%
% closed loop frequency domain analysis
S = inv(eye(2)+Gnom*Kmix2);
sigma(S, 'g', inv(Wp_S), 'r')

T = Gnom*Kmix2*S;
sigma(T)
sigma(T, 'g', inv(Wu_H2), 'r')

S = inv(eye(2)+Gnom*Kmix2);
sigma(S, 'g', inv(Wu_H8), 'r')

% KS
KS = Kmix2*S;
sigma(KS)

% poles of closed loop to verify pole placement objectives
pzplot((eye(2)+Gnom*Kmix2))
pole((eye(2)+Gnom*Kmix2))

% 
sigma(S, T)

%%
% using loopsens to make the loop instead of using the eqs

% sensitivity function. 
% max singular value is high at low freq, never below 0dB, so disturbance
% are likely to be amplified!
loops = loopsens(Gnom, Kmix2);
sigma(loops.So)

% 
impulse(loops.So, 1)

% good noise atten, 0dB ref tracking at low freq
loops = loopsens(Gnom, Kmix2);
sigma(loops.To)

step(loops.To, 1)


