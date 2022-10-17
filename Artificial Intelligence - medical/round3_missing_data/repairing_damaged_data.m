%%
% Try to repair the data, optionally
% done in lab ^_^

% Linear Fit
% poly1: Linear polynomial curve
% poly11: Linear polynomial surface
% poly2: Quadratic polynomial curve
% linearinterp: Piecewise linear interpolation
% cubicinterp: Piecewise cubic interpolation
% smoothingspline: Smoothing spline (curve)
% lowess: Local linear regression (surface)

% really crapp
y = cwkinputs(:,1);
f1 = fit([1:37]',y,'poly2', 'Exclude',y<1)
plot(f,[1:37]',cwkinputs(:,1))

% smoothing spline has some pretty harsh errors
y = cwkinputs(:,1);
f2 = fit([1:37]',y,'smoothingspline', 'Exclude',y<1)
plot(f,[1:37]',cwkinputs(:,1))

% cubic interpolation, great results!
% again, exclude zeros as they are wrong
y = cwkinputs(:,1);
f3 = fit([1:37]',y,'cubicinterp', 'Exclude',y<1)
plot(f,[1:37]',cwkinputs(:,1))

% summary of results
tfit = [1:37]';
plot(tfit,y,'r-',tfit,f1(tfit),'c--',tfit,f2(tfit),'k--',tfit,f3(tfit),'b--')
legend('Original Data','Polynomial Fit','Smoothing Spline','Cubic Interpolating Spline')

%%
% repair data using polynomial - developing the method!

% zero out a block for testing but preserve the original for comaprison!
cwkinputsorig = cwkinputs(1:37,8);
cwkinputs(16:20,8) = 0;
% take an example and analyse it
broken = cwkinputs(:,8)
% use a linear interpolation to get a curve
% if you get an error "index exceeds dimensions", delete the variable "fit"
y = broken;
f = fit([1:37]',y,'cubicinterp','Exclude',y<1)
%plot(f,[1:37]',broken)
% apply fix
for(i=1:37)
    if y(i)==0 
        fixed(i) = f(i);
    else
        fixed(i) = y(i);
    end
end
plot([1:37]',y','r--',[1:37]',fixed','b--',[1:37]',cwkinputsorig,'g--');
legend('broken','fixed','original')

cwkinputs(:,8)
% cwkinputsorig =
% 
%    87.0000
%   130.0000
%   118.6667
%   142.6667
%   180.0000
%   260.5000
%   265.3333
%   347.0000
%   382.6667
%   512.0000
%   570.3333
%   613.6667
%   655.3333
%   698.3334
%   688.0000
%   690.5000
%   699.8333
%   748.8333
%   709.0000
%   687.8333
%   711.3333
%   622.1667
%   645.0000
%   590.8333
%   625.3333
%   572.5000
%   632.8334
%   532.6667
%   548.0000
%   472.6667
%   505.5000
%   543.6667
%   451.0000
%   397.3333
%   488.0000
%   422.5000
%   325.3333

%%
% repair data using polynomial - applied!

% apply to all data
