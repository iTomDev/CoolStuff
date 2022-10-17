% DC Motor 
% 1. Nominal model
% 2. PI
% 4. Uncertain model
% 5. NCFSYN
% 6. LSDP

% Uses old style Matlab, mostly following that of Gu.
% DC motor

% Uncertainty Matrix Input: [deltau, u], Output: [deltay, y]

% v2.0
% + added comments to S and T on PI controller, loopsens
%

% Don't try to add h2hinf with old code, just do the model in new code if
% you really want an h2hinf DC motor.

%%

clc
clear


% nominal plant
A = [-R/L, -Kb/L;
     Km/J, -Kf/J];
B = [ 1/L 0]';
C = [0 1];
D = 0;
G = ss(A,B,C,D);
G

% pole/zero plot
pzplot(G)
pole(G)

% these are rubbish on matlab, dont waste your time, use bode instead
nyquist(G);

% bode
bode(G)

% root locus
rlocus(G(1,1))

%%
% PI controller design

clear
clc

% realistic values
% s = tf('s');
% J = 2.2e-6; % rotor moment of inertia [kg.m^2]
% Kf = 4.0e-6; % motor viscous friction constant [N.m.s]
% Kb = 0.0274; % emf constant [V/rad/sec]
% Km = 0.0274; % torque constant [N.m/Amp]
% R = 4; % motor resistance (terminal to terminal) [Ohms]
% L = 2.75e-6; % Electrical inductance (terminal to terminal) [Henry's]

% unrealistic values which match those from H8
s = tf('s');
J = 4; % rotor moment of inertia [kg.m^2] 
Kf = 3; % motor viscous friction constant [N.m.s] need new bearings haha ;)
Kb = 7; % emf constant [V/rad/sec]
Km = 1; % torque constant [N.m/Amp] 
R = 2; % motor resistance (terminal to terminal) [Ohms]
L = 5; % Electrical inductance (terminal to terminal) [Henry's] thats a lot of inductance :o

Gspeed = (Kb)/((J*s+Kf)*(L*s+R)+Km^2); % [rad/sec] / V
Gspeed

% PI controller for sensible values
% Kp = 1.0;
% Ki = 0.9;
% Kd = 0.00;
% K = tf([Kd Kp Ki],[1 0]);

% PI controller for the motor with 5 henries and fred flintstone bearings
% V1
Kp = 1.0;
Ki = 0.9;
Kd = 0.00;
K = tf([Kd Kp Ki],[1 0]);
CL = feedback(K*Gspeed,1);
step(CL)
rlocus(CL)
nyquist(CL)
%t = 0:0.0001:0.4
%step(CL,t)

% PI controller for the motor with 5 henries and fred flintstone bearings
% V2
Kp = 4;
Ki = 0.8;
Kd = 0;
K = tf([Kd Kp Ki],[1 0]);
CL = feedback(K*Gspeed,1);
step(CL)
rlocus(CL)
nyquist(CL)

% sensitivity function
L = K*Gspeed;
S = inv(1+L)
bode(S)

% complementary sensitivity function
T = (Gspeed*K)*S;
bode(T, 'b-', CL, 'r-')

% S alternative
% low S at low frequency indicates good disturbance attentuation since 
% y=S*d, and here S>>1 meaning d attentuates faster
loops = loopsens(Gspeed, K);
bode(loops.So)

% T alternative
% T is 0dB so reference tracking is probably about the same as the stock
% plant since y=T*r. Noise attenuation at high frequency should be good
% since T >> 1 at high frequency, and y=-T*n, so the effective attentuation
% of noise >> 1.
loops = loopsens(Gspeed, K);
bode(loops.To)

%%

% Method 2: Using sysic
% Donington park ^_^

% Uncertain plant. Manual uncertainty blocks, sysic interconnection
% original values
L_nom = 5;
R_nom = 2;
J_nom = 4;
Kb_nom = 7; % vicscous friction
Kf_nom = 3;
Km_nom = 1;
L_P = 0.1;
R_P = 0.4;
J_P = 0.6;
Kb_P = 0.3;
Kf_P = 0.3;
Km_P = 0.3;
ML = [0 L_nom; L_P L_nom];
MR = [0 R_nom; R_P R_nom];
MJ = [0 J_nom; J_P J_nom];
MKb = [0 Kb_nom; Kb_P Kb_nom];
MKf = [0 Kf_nom; Kf_P Kf_nom];
MKm = [0 Km_nom; Km_P Km_nom];
int1 = nd2sys(1, [1 0]);
int2 = nd2sys(1, [1 0]);

% a bit more realistic, not used as I would have to redo the design and
% diagrams!
% L_nom = 2.75e-6;
% R_nom = 4;
% J_nom = 2.2e-6;
% Kb_nom = 0.0274; 
% Kf_nom = 4.0e-6;
% Km_nom = 0.0274;
% L_P = 0.1;
% R_P = 0.4;
% J_P = 0.6;
% Kb_P = 0.3;
% Kf_P = 0.3;
% Km_P = 0.3;
% ML = [0 L_nom; L_P L_nom];
% MR = [0 R_nom; R_P R_nom];
% MJ = [0 J_nom; J_P J_nom];
% MKb = [0 Kb_nom; Kb_P Kb_nom];
% MKf = [0 Kf_nom; Kf_P Kf_nom];
% MKm = [0 Km_nom; Km_P Km_nom];
% int1 = nd2sys(1, [1 0]);
% int2 = nd2sys(1, [1 0]);

% use sysic to produce the uncertain open loop plant
% 1 system input, 1 system output, all others are purturbation inputs
systemnames = 'ML MR MJ MKb MKf MKm int1 int2';
sysoutname = 'G';
inputvar = '[uML; uMR; uMJ; uMKb; uMKf; uMKm; u]';
outputvar = '[ML(1); MR(1); MJ(1); MKb(1); MKf(1); MKm(1); int2]';
input_to_ML = '[uML; u-MR(2)-MKb(2)]';
input_to_int1 = '[ML(2)]';
input_to_MR = '[uMR; int1]';
input_to_MKm = '[uMKm; int1]';
input_to_MJ = '[uMJ; MKm(2)-MKf(2)]';
input_to_int2 = '[MJ(2)]';
input_to_MKf = '[uMKf; int2]';
input_to_MKb = '[uMKb; int2]';
sysic
G
minfo(G)

% open loop plant for simulating closed loop using sysic
systemnames = ' G ';
inputvar = '[pert{6}; ref; dist; control]';
outputvar = '[G(1:6); G(7)+dist; ref - G(7) - dist]';
input_to_G = '[pert; control]';
sysoutname = 'simol_ic';   %sim_ic
cleanupsysic = 'yes';
sysic;
simol_ic

%%
% Open loop plant analysis

% plot perturbed open loop frequency response (old code, mostly Gu)
omega = logspace(-2,4,1000);
[delta1,delta2,delta3,delta4,delta5,delta6] = ndgrid([-1 0 1],[-1 0 1],[-1 0 1],[-1 0 1],[-1 0 1],[-1 0 1]);
for j=1:27
    delta = diag([delta1(j),delta2(j),delta3(j),delta4(j),delta5(j),delta6(j)]);
    olp = starp(delta,G);
    olp_ic = sel(olp,1,1);      % starp(Delta,G) has 1 in, 1 out
    olp_g = frsp(olp_ic,omega);
    figure(1)
    vplot('bode',olp_g,'c-');
    subplot(2,1,1)
    hold on
    subplot(2,1,2)
    hold on
end
% plot nominal open loop frequency response
subplot(2,1,1)
olp_ic = sel(G,7,7);        % the system input and output of G are (7 to 7)
olp_g = frsp(olp_ic,omega);
vplot('bode',olp_g,'r--');
subplot(2,1,1)
title('Bode plots of perturbed plants');
hold off
subplot(2,1,2)
hold off
stpdg = (starp(delta,G));
minfo(stpdg)

% singular values of G
olp_ic = sel(G,7,7);
olp_ic
sigma(olp_ic, omega)

% transient response
% olp_ic = sel(G,7,7);
% olp_ic
% %clp = starp(olp_ic, 1);
% timedata = [0 20 40];
% stepdata = [1 0 1];
% dist = 0;
% ref = step_tr(timedata, stepdata, 0.1, 60);
% u = abv(0,0,0,ref,dist);
% u
% y = trsp(olp_ic,u,60,0.1);
% figure(1)
% vplot(sel(y,4,1), 'y', ref, 'r--')

% convert to modern code to use lsim
minfo(olp_ic) % 2 states     1 outputs     1 inputs
[acl bcl ccl dcl] = unpck(olp_ic);
olp_ic2 = ss(acl,bcl,ccl,dcl);
size(bcl) % 
size(ccl) % 

% sim cl - reference tracking (need to ditch one y axis side)
t = 0:0.01:6;
sizet = size(t);
r = zeros(1,sizet(2));        % 
r(1,:) = 1*ones(size(t));  % input
r;
[y,t,x] = lsim(olp_ic2,r,t);
[AX,H1,H1] = plotyy(t, y(:,1), t, y(:,1), 'plot');
set(get(AX(1),'YLabel'), 'String','Motor Speed [rad/sec]')
xlabel('Time [Seconds]');
title('Open Loop Time Response')

% pole/zero plot
pzplot(olp_ic2)

% nyquist of nominal plant
nyquist(olp_ic2)

% nyquist with an uncertainty circle

% root locus
rlocus(olp_ic2)

%%
% Design some weights and create the open loop weighted plant with sysic

% weights should have: 
% - a decade or two in the trasition band
% - no more than -20dB/dec in trasition band
% - proper, min phase. #zeros < #poles
% - minSV(L) > Wp (at low omega)
% - maxSV(L) < Wu (at high omega)
% - Lo = G*K and Li = K*G should both meet those loop specs (Hard to do!)

% Wp = nd2sys([1 1.8 10], [1 8 0.01]); % 1st try. GAM = 13.0043
% Wu = nd2sys([1], [1]);  

Wp = nd2sys([1 1.8 10], [1 8 0.01]); % GAM = 1.3125
Wu = nd2sys([.1], [1]);  

% singular values of inverse performance weight
% maxSV(I+GK)^-1 < 1/Wp
omega = logspace(-2,4,1000); %omega = logspace(-4,4,100);
Wp_g = frsp(Wp, omega);
Wpi_g = minv(Wp_g);
vplot('liv,lm', Wpi_g);
title('Inverse Performance Weight');
xlabel('Freq [rad/sec]');
ylabel('Mag [dB]');

% singular values of open loop plant G and inverse performance weight
omega = logspace(-2,4,1000); %omega = logspace(-4,4,100);
Wp_g = frsp(Wp, omega);
Wpi_g = minv(Wp_g);
G_g = frsp(sel(G,7,7), omega);
vplot('liv,lm', G_g, 'b-', Wpi_g, 'g-');
title('G vs Inverse Performance Weight');
xlabel('Freq [rad/sec]');
ylabel('Mag [dB]');

% singular values of open loop plant G, robustness weight, inv performance weight
omega = logspace(-2,4,1000); % omega = logspace(-4,4,100);
Wp_g = frsp(Wp, omega); % inv perf weight
Wpi_g = minv(Wp_g); % 
G_g = frsp(sel(G,7,7), omega); % plant G
Wu_g = frsp(Wu, omega); % plant G
vplot('liv,lm', G_g, 'b-', Wpi_g, 'g-', Wu_g, 'r-');
title('G vs Robust weight and (Performance Weight)^-1');
xlabel('Freq [rad/sec]');
ylabel('Mag [dB]');

% Inputs: perturbations; disturbance; controller outputs
% Outputs: perturbations; Performance weight; uncertainty weight; output + disturbance
systemnames = 'G Wp Wu';
inputvar = '[pert{6}; dis t; control]';
outputvar = '[G(1:6); Wp; -Wu; -G(7)-dist]';
input_to_G = '[pert; control]';
input_to_Wp = '[G(7)+dist]';
input_to_Wu = '[control]';
sysoutname = 'wolp_ic'; %sys_ic
cleanupsysic = 'yes';
sysic;
wolp_ic
minfo(wolp_ic)

% singular values of weighted open loop plant, robustness weight, inv performance weight
omega = logspace(-2,4,1000); % omega = logspace(-4,4,100);
Wp_g = frsp(Wp, omega); % inv perf weight
Wpi_g = minv(Wp_g); % 
wolp_ic_g = frsp(sel(wolp_ic,9,8), omega); % plant wolp_ic
Wu_g = frsp(Wu, omega); % uncertainty weight
vplot('liv,lm', wolp_ic_g, 'b-', Wpi_g, 'g-', Wu_g, 'r-');
title('Weighted Plant vs Weights');
xlabel('Freq [rad/sec]');
ylabel('Mag [dB]');

%%
% synthesis controller using hinfsyn

hin_ic = sel(wolp_ic, [7:9], [7:8]); % select the non perturbation IO (output, input)
nmeas = 1; ncont = 1; gmin = 1; gmax = 1000; tol = 0.01;
[K clp] = hinfsyn(hin_ic, nmeas, ncont, gmin, gmax, tol); % inputs: u, outputs: y

%%

% form closed loop plant
clp_h8 = starp(simol_ic, K);
minfo(clp_h8)
% 6 states     7 outputs     8 inputs

% singular values of closed loop plant and inverse performance weight
% plant should be below the inverse weight (blue below green)
omega = logspace(-2,4,1000); % omega = logspace(-4,4,100);
Wp_g = frsp(Wp, omega);
Wpi_g = minv(Wp_g);
clp_h8_g = frsp(sel(clp_h8,7,8), omega);          % control to ref-G-dist
vplot('liv,lm', clp_h8_g, 'b-', Wpi_g, 'g-');
title('Closed Loop Plant vs Inverse Performance Weight');
xlabel('Freq [rad/sec]');
ylabel('Mag [dB]');

% time response (old code)
% timedata = [0 20 40];
% stepdata = [1 0 1];
% dist = 0;
% ref = step_tr(timedata, stepdata, 0.1, 60);
% u = abv(0,0,0,0,0,0,ref,dist); % 0's are uncertainty inputs
% y = trsp(clp,u,60,0.1);
% figure(1)
% vplot(sel(y,4,1), 'y', ref, 'r--')

% transform to modern code for time response
minfo(clp_h8) %6 states     7 outputs     8 inputs
[acl bcl ccl dcl] = unpck(clp_h8);
clp_h8_2 = ss(acl,bcl,ccl,dcl);
size(bcl) 
size(ccl) 

% plot reference tracking time response of hinfsyn plant
t = 0:0.01:2;
sizet = size(t);
r = zeros(8,sizet(2));        % 8*t array
r(7,:) = 1*ones(size(t));  % step into channel 7 - reference
r;
[y,t,x] = lsim(clp_h8_2,r,t);
[AX,H1,H1] = plotyy(t, y(:,7), t, y(:,7), 'plot');
set(get(AX(1),'YLabel'), 'String','Motor Speed [rad/sec]')
xlabel('Time [Seconds]');
title('Closed Loop Reference Tracking of hinfsyn Controller')

% plot disturbance rejection time response of hinfsyn plant
t = 0:0.01:2;
sizet = size(t);
r = zeros(8,sizet(2));        % 8*t array
r(8,:) = 1*ones(size(t));  % step into channel 7 - reference
r;
[y,t,x] = lsim(clp_h8_2,r,t);
[AX,H1,H1] = plotyy(t, y(:,7), t, y(:,7), 'plot');
set(get(AX(1),'YLabel'), 'String','Motor Speed [rad/sec]')
xlabel('Time [Seconds]');
title('Closed Loop Disturbance Rejection of hinfsyn Controller')

%%
% mu analysis of hinfsyn controller

% compute mu bounds as there is strutured uncertainty
% problem 1: matin dimensions not compatible
% solution 1: use clp2_g = frsp(clp2, omega); with all rows and columns,
% dont select any
% problem 2: lower mu bound is higher than upper mu bound
%clp_lsdp
omega = logspace(-2,4,100); % omega = logspace(-5,5,100);
clp2 = starp(wolp_ic, K);
minfo(wolp_ic) % 4 states     9 outputs     8 inputs
minfo(clp2) %  8 states     8 outputs     7 inputs
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

%%
% mu synthesis

DK_DEF_NAME = 'dk_mds';
dkit
K_mu = k_dk4mds;
K_mu

% close loop mu synth plant

% closed loop mu plant
omega = logspace(-2,4,1000); % omega = logspace(-4,4,100);
clp_mu = starp(simol_ic, K_mu); %6 states     7 outputs     8 inputs
minfo(clp_mu)
clp_mu_g = frsp(sel(clp_mu,7,8), omega);          % control to ref-G-dist
%Wp_g = frsp(Wp, omega);
%Wpi_g = minv(Wp_g);
vplot('liv,lm', clp_mu_g)
title('Frequency response of Mu Synthesis closed loop');
xlabel('Freq [rad/sec]');
ylabel('Mag [dB]');

% transform to modern code
minfo(clp_mu) %6 states     7 outputs     8 inputs
[acl bcl ccl dcl] = unpck(clp_mu);
clp_mu2 = ss(acl,bcl,ccl,dcl);
size(bcl) 
size(ccl) 

% plot reference tracking time response of mu synth plant
t = 0:0.01:2;
sizet = size(t);
r = zeros(8,sizet(2));        % 8*t array
r(7,:) = 1*ones(size(t));  % step into channel 7 - reference
r;
[y,t,x] = lsim(clp_mu2,r,t);
[AX,H1,H1] = plotyy(t, y(:,7), t, y(:,7), 'plot');
set(get(AX(1),'YLabel'), 'String','Motor Speed [rad/sec]')
xlabel('Time [Seconds]');
title('Closed Loop Reference Tracking of mu Synthesis Controller')

% plot disturbance rejection time response of mu synth plant
t = 0:0.01:2;
sizet = size(t);
r = zeros(8,sizet(2));        % 8*t array
r(8,:) = 1*ones(size(t));  % step into channel 8 - disturbance
r;
[y,t,x] = lsim(clp_mu2,r,t);
[AX,H1,H1] = plotyy(t, y(:,7), t, y(:,7), 'plot');
set(get(AX(1),'YLabel'), 'String','Motor Speed [rad/sec]')
xlabel('Time [Seconds]');
title('Closed Disturbance Rejection of mu Synthesis Controller')

%%
%%
%%


% loop shaping design

% A
% W1 = nd2sys([1], [1 1], 50); % GAM = 1.4057
% W2 = 1;%nd2sys([1], [.1]);  

% B
% W1 = nd2sys([1], [2 0], 50); % GAM = 1.5259
% W2 = 1;%nd2sys([1], [.1]);  

% C
% W1 = nd2sys([3], [2 0], 50); % GAM = 1.7278, better response but more gain 
% W2 = 1;%nd2sys([1], [.1]);  

% D
% W1 = nd2sys([3], [2 0], 20); % GAM = 1.5475, worse response, less gain
% W2 = 1;%  

% E
% W1 = nd2sys([3], [2 0], 20); % GAM = 2.3377, much faster response
% W2 = nd2sys([1], [.1]);  

% F
W1 = nd2sys([3], [2 0], 1); % GAM = 1.9773, unity gain :]
W2 = nd2sys([1], [.01]);  

% frequency response of weight1
omega = logspace(-2,4,1000); % omega = logspace(-2,4,100);
W1_g = frsp(W1, omega);
vplot('liv,lm', W1_g, 'r-');
title('Frequency response of the precompensator W1');
xlabel('Freq [rad/sec]');
ylabel('Mag [dB]');

% form shaped plant
G_ls = sel(G,7,7) % sel(output,input)
Gs= mmult(W2, G_ls, W1);  % Gs = w2*G*w1

% frequency response of shaped plant and weight
omega = logspace(-2,4,1000); % omega = logspace(-2,4,100);
Gs_g = frsp(Gs, omega);
vplot('liv,lm', Gs_g, 'r-');
title('Frequency response of the shaped plant');
xlabel('Freq [rad/sec]');
ylabel('Mag [dB]');

% synthesis controller
% [sysk,emax,sysobs]=ncfsyn(sysgw,factor,opt)
[Klsh, emax] = ncfsyn(Gs, 1.1);
emax      % stability margin
inv(emax) % gamma iirc

% construct negative feedback controller
K = mmult(W1, Klsh, W2);  % K = w1*Klsh*w2
[ak bk ck dk] = unpck(K);
ck = -ck;
dk = -dk;
Klsdp = pck(ak,bk,ck,dk);

% closed loop plant
omega = logspace(-2,4,1000); % omega = logspace(-4,4,100);
clp_lsdp = starp(simol_ic, Klsdp);
minfo(clp_lsdp) % 6 states     7 outputs     8 inputs
clp_lsdp_g = frsp(sel(clp_lsdp,7,8), omega);          % control to ref-G-dist
%Wp_g = frsp(Wp, omega);
%Wpi_g = minv(Wp_g);
vplot('liv,lm', clp_lsdp_g)
title('Frequency response of LSDP Closed Loop');
xlabel('Freq [rad/sec]');
ylabel('Mag [dB]');


%%
% draw some closed loop graphs

% DELETE
% time response to reference (doublet)
% timedata = [0 20 40];
% stepdata = [1 0 1];
% dist = 0;
% ref = step_tr(timedata, stepdata, 0.1, 60);
% u = abv(0,0,0,0,0,0,ref,dist); % 0's are uncertainty inputs
% y = trsp(clp_lsdp,u,60,0.1);
% figure(1)
% vplot(sel(y,4,1), 'y', ref, 'r--')
% time response to disturbance (doublet)
% timedata = [0 20 40];
% stepdata = [1 0 1];
% dist = step_tr(timedata, stepdata, 0.1, 60);
% ref = 0;
% u = abv(0,0,0,0,0,0,ref,dist); % 0's are uncertainty inputs
% y = trsp(clp_lsdp,u,60,0.1);
% figure(2)
% vplot(sel(y,4,1), 'y', dist, 'r--')


% convert to modern code to use lsim
% inputs: pert{6}; ref; dist; = 8
% outputs: pert{6}; G(7)+dist = 7
% starp closes the loop from (ref - G(7) - dist) to control, so 1 fewer
% input and output.
% input 7 is reference
% input 8 is disturbance
% outut 7 is y with a disturbance added, aka int2 with noise
minfo(clp_lsdp) %6 states     7 outputs     8 inputs
[acl bcl ccl dcl] = unpck(clp_lsdp);
clp_lsdp2 = ss(acl,bcl,ccl,dcl);
size(bcl) % 6x8 = 8 inputs
size(ccl) % 7x6 = 7 outputs

% sim cl - reference tracking (need to ditch one y axis side)
t = 0:0.01:2;
sizet = size(t);
r = zeros(8,sizet(2));        % 8*t array
r(7,:) = 1*ones(size(t));  % step into channel 7 - reference
r;
[y,t,x] = lsim(clp_lsdp2,r,t);
[AX,H1,H1] = plotyy(t, y(:,7), t, y(:,7), 'plot');
set(get(AX(1),'YLabel'), 'String','Motor Speed [rad/sec]')
xlabel('Time [Seconds]');
title('Closed Loop Reference Tracking of LSDP Controller')

% sim cl - disturbance rejection (need to ditch one y axis side)
t = 0:0.01:2;
sizet = size(t);
r = zeros(8,sizet(2));        % 8*t array
r(8,:) = 1*ones(size(t));  % step into channel 7 - reference
r;
[y,t,x] = lsim(clp_lsdp2,r,t);
[AX,H1,H1] = plotyy(t, y(:,7), t, y(:,7), 'plot');
set(get(AX(1),'YLabel'), 'String','Motor Speed [rad/sec]')
xlabel('Time [Seconds]');
title('Closed Loop Disturbance Rejection of LSDP Controller')

% reference tracking of a square wave
samplet = 0.01; 
period = 1; % period of wave
final = 10;    % final time point of wave
[u,t] = gensig('square',period,final,samplet); % generate square wave
t = 0:samplet:final;
sizet = size(t);
r = zeros(8,sizet(2));        % 8*t array
r(7,:) = u(:);  % put the signal into channel 7
[y,t,x] = lsim(clp_lsdp2,r,t);
[AX,H1,H1] = plotyy(t, y(:,7), t, y(:,7), 'plot');
set(get(AX(1),'YLabel'), 'String','Motor Speed [rad/sec]')
xlabel('Time [Seconds]');
title('Closed Loop Square Wave Reference Tracking of LSDP Controller')

% reference tracking of a sine wave
samplet = 0.01; 
period = 1; % period of wave
final = 10;    % final time point of wave
[u,t] = gensig('sin',period,final,samplet); % generate square wave
t = 0:samplet:final;
sizet = size(t);
r = zeros(8,sizet(2));        % 8*t array
r(7,:) = u(:);  % put the signal into channel 7
[y,t,x] = lsim(clp_lsdp2,r,t);
[AX,H1,H1] = plotyy(t, y(:,7), t, y(:,7), 'plot');
set(get(AX(1),'YLabel'), 'String','Motor Speed [rad/sec]')
xlabel('Time [Seconds]');
title('Closed Loop Square Wave Reference Tracking of LSDP Controller')

%%
%%
% mu analysis of hinfsyn controller

% compute mu bounds as there is strutured uncertainty
% problem 1: matin dimensions not compatible
% solution 1: use clp2_g = frsp(clp2, omega); with all rows and columns,
% dont select any
% problem 2: lower mu bound is higher than upper mu bound
%clp_lsdp
omega = logspace(-2,4,100); % omega = logspace(-5,5,500);
clp_lsdp = starp(wolp_ic, K);
clp_lsdp_g = frsp(clp_lsdp, omega);  
blockrsR = [-1 1; -1 1; -1 1; -1 1; -1 1; -1 1]; % 6 real uncertainties
rob_stab = sel(clp_lsdp_g, [1:6], [1:6]); % uncertaint IOs 1-6
pdim = ynum(rob_stab);                  % pdim = 1
fixl = [eye(pdim); 0.01*eye(pdim)]; 
fixr = fixl';
blkrs = [blockrsR; abs(blockrsR)];
clp_mix = mmult(fixl, rob_stab, fixr);
minfo(blkrs) 
minfo(clp_mix)  
[rbnds,rowd,sens,rowp,rowg] = mu(clp_mix, blkrs);  
vplot('liv,lm', sel(rbnds,1,1),'r-'...   % dashed - probably the upper
                ,sel(rbnds,1,2),'g--'...   % solid - probably the lower
                ,vnorm(rob_stab),'c-.')   % h8 norm bound 
title('mu Analysis: Robust Stability');
xlabel('Frequency [rad/sec]')
ylabel('mu');

%%
%%

% H8/H2 mixed synthesis - DOESNT WORK


%[gopt,h2opt,K,R,S] = hinfmix(P,r,obj,region,dkbnd,tol)
% P: any SS, TF or ZPK
% r: three entry vector: [length of z2, length of y, length of u]. z2
% obj: [h8 constraint, h2 constraint, trade off criterion alpha and beta]
% none would be [0 0 0 0]
% gopt: h8 perf, h2opt: h2 perf, R and S: Optimal LMI variables 
% region: LMI region for pole placement. Use lmireg to create, def is LHP
% dkbnd: norm of controller feedthrough max. Def is 100, use 0 to get
% strictly proper K
% tol defaul: 10^-2


[gopt,h2opt,K,R,S] = hinfmix(wolp_ic,[0 1 1],[.001 0 0 0])
clp_mixsyn = starp(simol_ic, K);
minfo(clp_mixsyn)

minfo(clp_mixsyn) %6 states     7 outputs     8 inputs
[acl bcl ccl dcl] = unpck(clp_mixsyn);
clp_mixsyn2 = ss(acl,bcl,ccl,dcl);
size(bcl) % 6x8 = 8 inputs
size(ccl) % 7x6 = 7 outputs

% sim cl - reference tracking (need to ditch one y axis side)
t = 0:0.01:2;
sizet = size(t);
r = zeros(8,sizet(2));        % 8*t array
r(7,:) = 1*ones(size(t));  % step into channel 7 - reference
r;
[y,t,x] = lsim(clp_mixsyn2,r,t);
[AX,H1,H1] = plotyy(t, y(:,7), t, y(:,7), 'plot');
set(get(AX(1),'YLabel'), 'String','Motor Speed [rad/sec]')
xlabel('Time [Seconds]');
title('Closed Loop Reference Tracking of H2/Hinf Controller')


%%
%%



%%
% comparison graphs

% NEXT: 
% 2. Do paper diags of the plants sysics (easy!)
% 3. carry on writeup of the graphs you have so far, that would be enough for 1
% plant report which you can send off (yay!)
% 4. Add a Nyquist and circles
% popov?

% closed loop plant
omega = logspace(-2,4,1000); % omega = logspace(-4,4,100);
clp_lsdp = starp(simol_ic, Klsdp);
minfo(clp_lsdp) % 6 states     7 outputs     8 inputs
clp_h8_g = frsp(sel(clp_h8,7,8), omega);          % control to ref-G-dist
clp_lsdp_g = frsp(sel(clp_lsdp,7,8), omega);          % control to ref-G-dist
clp_mu_g = frsp(sel(clp_mu,7,8), omega);          % control to ref-G-dist
%Wp_g = frsp(Wp, omega);
%Wpi_g = minv(Wp_g);
vplot('liv,lm', clp_h8_g, 'r-', clp_mu_g, 'b-', clp_lsdp_g, 'g-')
title('Closed Loop Frequency Response of hinf, mu and LSDP');
xlabel('Freq [rad/sec]');
ylabel('Mag [dB]');

% reference tracking of all 3
% hinfinity
t = 0:0.01:2;
sizet = size(t);
r = zeros(8,sizet(2));        % 8*t array
r(7,:) = 1*ones(size(t));  % step into channel 7 - reference
r;
[y1,t,x] = lsim(clp_h8_2,r,t);

% mu
t = 0:0.01:2;
sizet = size(t);
r = zeros(8,sizet(2));        % 8*t array
r(7,:) = 1*ones(size(t));  % step into channel 7 - reference
r;
[y2,t,x] = lsim(clp_mu2,r,t);

% lsdp
t = 0:0.01:2;
sizet = size(t);
r = zeros(8,sizet(2));        % 8*t array
r(7,:) = 1*ones(size(t));  % step into channel 7 - reference
r;
[y3,t,x] = lsim(clp_lsdp2,r,t);

plot(t, y1(:,7), 'r-', t, y2(:,7), 'b-', t, y3(:,7), 'g-')
ylabel('Motor Speed [rad/sec]');
xlabel('Time [Seconds]');
title('Closed Loop Reference Tracking of hinf, mu and LSP')














