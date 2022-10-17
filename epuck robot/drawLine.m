function [  ] = drawLine(x1,y1,x2,y2,colour)
%LINE Draws a line
% I'll be needing these graphics primatives to make that potential fields
% stuff work!
% 5.10.2017

x = [x1 x2];
y = [y1 y2];
line(x,y,'Color',colour,'LineStyle','-');
end

