
% load matrix vals
load('A1B1.mat')


% run sim in simulink now

%%
% plot vals
% just y1
plot(Yout(:,1));
% plot y1, y2, and u
plot([0:1:600]',Yout(:,1),'b--',[0:1:600]',Yout(:,2),'k--',[0:1:600]',Uout(:),'g--');
plot(tout(:),Yout(:,1),'b-',tout(:),Yout(:,2),'k-',tout(:),Uout(:),'g-');
title('Step response')
legend('y1','y2','u')

% can use output error or least squares approache to obtain the linear model

%%
% help oe
%{
help oe
 oe  Estimate Output-Error polynomial model using time or frequency domain data.
 
   M = oe(Z, [nb nf nk])
     estimates an Output Error model represented by:
       y(t) = [B(q)/F(q)] u(t-nk) +  e(t)
     where:
        nb = order of B polynomial + 1 (Ny-by-Nu matrix)
        nf = order of F polynomial     (Ny-by-Nu matrix)
        nk = input delay (in number of samples, Ny-by-Nu matrix)
        (Nu = number of inputs, Ny = number of outputs)
 
     The estimation may be performed using either time or frequency domain
     data. The estimated model is delivered as an @idpoly object. M
     contains the estimated values for B and F polynomials along with their
     covariances and structure information.
 
     Z is the estimation data given as an IDDATA or an IDFRD object. Use
     IDDATA object for input-output signals (time or frequency domain). Use
     FRD or IDFRD object for frequency response data. Type "help iddata"
     and "help idfrd" for more information. nb, nf and nk are the
     polynomial orders associated with the oe model.
 
   M = oe(Z, [nb nf nk], 'Name1', Value1, 'Name2', Value2,...)
     specifies additional attributes of the model structure as name-value
     pairs. Specify as one or more of the following:
     'InputDelay': Specify input delay as a double vector of length equal
                  to number of inputs. For continuous-time models, use a
                  vector of real numbers. For discrete-time models, value
                  must be an integer vector denoting the delay as multiples
                  of sample time.
       'ioDelay': Input-to-output delay (double matrix). For
                  continuous-time models, specify as an Ny-by-Nu matrix of
                  nonnegative numbers. For discrete-time models, the value
                  must be an integer matrix denoting the delays as
                  multiples of sample time; in discrete-time case, this is
                  useful as a replacement for "nk" order - max(nk-1,0) lags
                  can be factored out as "ioDelay" value.
 
   M = oe(Z, [nb nf nk], ..., OPTIONS)
     specifies estimation options that configure the estimation objective,
     initial conditions and numerical search method to be used for
     estimation. Use the "oeOptions" command to create the option set
     OPTIONS.
 
   M = oe(Z, M0)
   M = oe(Z, M0, OPTIONS)
     uses the IDPOLY model M0 to configure the initial parameterization of
     the resulting model M. M0 must be a model of oe structure (only A,
     B and C polynomials must be active). M0 may be created using the
     IDPOLY constructor or could be the result of a previous estimation.
     The initial model argument, M0, may be followed by estimation options
     to configure estimation options. If OPTIONS is not specified and M0
     was created by estimation, the options are taken from M0.Report.OptionsUsed.
 
    Continuous Time Model Estimation
    It is recommended to use TFEST for continuous-time estimations.
    However, when using continuous-time frequency domain data (IDDATA or
    IDFRD with Ts=0), continuous-time models are also estimated using oe.
    Then 'nk' should be omitted from orders: M = oe(Z,[nb nf]).
 
    Example: Fit continuous-time transfer function to frequency response.
    1. Generate data:
        SYS = tf([1 3],[1 2 1 1]); % TF requires Control System Toolbox
        G = idfrd(SYS,logspace(-2,2,256)); % continuous-time FR data
    2. Estimate an oe model to fit the data G
        M = oe(G, [2 3]); % use syntax oe(DATA, [nb, nf])
        bode(G, M) % compare data to model
 
   NOTE: Output Error models are a special configuration of IDPOLY models
   that have only two active polynomials - B and F. For such models, it may
   be more convenient to use IDTF model object and the associated
   estimation command TFEST.
 
    See also oeOptions, tfest, arx, armax, iv4, n4sid, bj, polyest, idpoly,
    iddata, idfrd, idParametric/sim, compare.
%}

%%
% apply oe to fit a model
nb = 2;
nf = 3;
nk = 0; 
Z = Yout;
M = oe(Z, [nb nf nk])

%%
% compare model to original data
TM = step(M)
%plot(tout(:),Yout(:,1),'b-',tout(:),Yout(:,2),'k-',tout(:),Uout(:),'g-');

%%
% tims code to create a model

z1 = iddata(Yout(:,1),Uout,0.025);
order = [1 1 1];
m1 = oe(z1,order)
% Discrete-time OE model:  y(t) = [B(z)/F(z)]u(t) + e(t)
% B(z) = 2.184 z^-40                                                                                  
% F(z) = 1 -0.9828 z^-1 
% Fit to estimation data: 94.4%  

z2 = iddata(Yout(:,2),Uout,0.025);
order = [1 1 1];
m2 = oe(z2,order)
% Discrete-time OE model:  y(t) = [B(z)/F(z)]u(t) + e(t)
% B(z) = 3.583 z^-40                                  
% F(z) = 1 -0.9823 z^-1      
% Fit to estimation data: 96.34%

% convert to continuous form
d2c(m1)
% 88.12/s+0.6958
d2c(m2)
% 144.6 /  s+0.7161 

s = tf('s');
m1 = c2d(tf(88.12,[1 0.6958]),0.025);
%     2.184
%   ----------
%   z - 0.9828
m2 = c2d(tf(144.6,[1 0.7161]),0.025);
%     3.583
%   ----------
%   z - 0.9823

%%
% compare fitted model1 to y1 data
% need to add a 1 sec delay if you want to plot like this.
% also better to do it as discrete response
figure;
m1step = step(m1)
szm1step = size(m1step,1)
subplot(1,2,1)
plot([0:1:szm1step-1]',m1step(:),'b--',[0:1:szm1step-1]',Yout(1:szm1step,1),'k--');

% compare fitted model2 to y2 data
opt = stepDataOptions('StepAmplitude',1);
m2step = step(m2)
szm2step= size(m2step,1);
subplot(1,2,2)
plot([0:1:szm2step-1]',m2step(:),'b--',[0:1:szm2step-1]',Yout(1:szm2step,2),'k--');



