function [ output_args ] = drawHouse(xdes, ydes)
%DRAWHOUSE Summary of this function goes here
%   Detailed explanation goes here

x = [0  0 15 15 0];
y = [0 10 10  0 0];
% translate
x = x+xdes;
y = y+ydes;
plot(x(:), y(:), 'k-', 'LineWidth', 1);

end

