

%%
% Problem: During hinfsyn, "not a system". 
% Solution: Use lftdata and ss to convert the uss to a system
% Problem: D21 not full row rank
% Solution: Double check sysic plant, there is probably a connection that
% makes no sense
%
%
% 1. Model
clear
clc

%m = ureal('m', 0.8, 'percentage', 10);    % Mass
%k = ureal('k', 0.1, 'percentage', 10);    % Spring constant
%c = ureal('c', 0.5, 'percentage', 10);    % Damper
k = 0.2;
c = 0.4;
m = 0.1;

% Nominal model state equations
A = [ 0    1; 
     -k/m -c/m];
B = [0;
     1/m];
C = [1 0]; 
D = 0;
G = ss(A,B,C,D);

% obtain system matrix of uss G
[M Delta] = lftdata(G);
[Ma Mb Mc Md] = ssdata(M);
Glft = ss(Ma, Mb, Mc, Md);
Glft

% lft of ureal m
lftdata(m)
%          0    0.2828
%     0.2828    0.8000

% lft of ureal k
lftdata(k)
%          0    0.1000
%     0.1000    0.1000

% lft of ureal c
lftdata(c)
%          0    0.2236
%     0.2236    0.5000


%%
% 2. Analysis

%%
% 3. Weight plant
% form open loop plant
% choose some weights
% works, needs designing

% In H8 design we have objectives defined in terms of the closed loop
% transfer functions S and T, but we must formulate these requirements as
% open loop tranfer function objectives using the plant weights.

% weights need to roll off before the first open loop zero! so find it

W1 = ss(tf([1 1.8 10],[1 8 .01]));
W2 = ss(tf(1,0.5));


% W1 = makeweight(.1, 20, 50); % dc gain %, cutoff rad/sec, final gain %
% W2 = makeweight(.2, 45, 50);

%ref= 0.5;

%open loop model
systemnames = 'G W1 W2';
inputvar = '[ref]';
outputvar = '[W2]';
input_to_W1 = '[ref-W2]';
input_to_G = '[W1]';
input_to_W2 = '[G]';
sysoutname = 'wplant';
%cleanupsysic = 'yes';
sysic;
wplant

% obtain system matrix for hinfsyn
% [M Delta] = lftdata(wplant);
% [Ma Mb Mc Md] = ssdata(simplify(M));
% wplant1 = ss(Ma, Mb, Mc, Md);
% wplant1

% create a controller
% [k,g,gfin] = hinfsyn(p,nmeas,ncon,gmin,gmax,tol,ricmethd,epr,epp
nmeas = 1; % row dim of C2
ncont = 1; % column dim of B2
%[Khinf stuff more] = hinfsyn(wplant1, nmeas, ncont, 0.1, 10, .001);
[K CL INFO] = hinfsyn(wplant, nmeas, ncont);
K
CL
INFO



%%

% Alternative weighting configuration
% weights haven't been designed
% works, needs designing

Wu = makeweight(.1, 20, 50); % dc gain %, cutoff rad/sec, final gain %
Wp = makeweight(.2, 45, 50);
bode(Wu)
grid
bode(Wp)
grid

Wu = ss(tf([1 1.8 10],[1 8 .01]));

Wp = ss(tf(1,1, 10^(-2)));

Wd = 0.1;

Wn = 0.1;

%open loop model
systemnames = 'G Wu Wp Wd Wn';
inputvar = '[dist; noise; ref]';
outputvar = '[Wu; Wp; G+Wd+Wn]';
input_to_Wu = '[ref]';
input_to_G = '[ref]';
input_to_Wd = '[dist]';
input_to_Wp = '[G+Wn]';
input_to_Wn = '[noise]';
sysoutname = 'wplant';
%cleanupsysic = 'yes';
sysic;
wplant

% obtain system matrix for hinfsyn
[M Delta] = lftdata(wplant);
[Ma Mb Mc Md] = ssdata(M);
wplant1 = ss(Ma, Mb, Mc, Md);
wplant1

% create a controller
% [k,g,gfin] = hinfsyn(p,nmeas,ncon,gmin,gmax,tol,ricmethd,epr,epp
nmeas = 1;
ncont = 1;
[Khinf] = hinfsyn(wplant1, nmeas, ncont, 0.1, 10, .001);

%% 
