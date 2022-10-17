function [ output_args ] = drawPolygon(x,y )
%DRAWPOLYGON Summary of this function goes here
%   Detailed explanation goes here

%x = [1 2 3];
%y = [1 2 3];
for i=1:1:(sel(size(x),1,2)-1)
    drawLine(x(i),y(i),x(i+1),y(i+1),'k')
end
axis([0 10 0 10]);
