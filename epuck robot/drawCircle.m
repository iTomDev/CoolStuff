function [ h ] = drawCircle( x,y,r)
%CIRCLE Summary of this function goes here
%   https://uk.mathworks.com/matlabcentral/answers/98665-how-do-i-plot-a-circle-with-a-given-radius-and-center
% 5.10.2017

d = r*2;
px = x-r;
py = y-r;
h = rectangle('Position',[px py d d],'Curvature',[1,1]);
daspect([1,1,1]);

end

