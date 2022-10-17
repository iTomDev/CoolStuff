function [] = drawRiver(desx, desy)

% first part of river
%x = [0 0 12 24 42 48 52 64 82 100 100 84 68 62 56 54 48 40 26 18 14 0];
%y = [0 3  3  5  5  4  4  3  4   4   1  1  0  0  1  1  1  2  2  1  0 0];
x = [0 0 12 24 30 30 18 14 0];
y = [0 3  3  5  5 2  1  0 0];
% translate
y = y+30;
plot(x(:), y(:), 'b-', 'LineWidth', 1);

x = [40 48 52 64 80 80 68 62 56 54 48 40];
y = [5  4  4  3  4   1  0  0  1  1  1  2];
% translate
y = y+30;
plot(x(:), y(:), 'b-', 'LineWidth', 1);

x = [90 100 100 90];
y = [4    4   1  1];
% translate
y = y+30;
plot(x(:), y(:), 'b-', 'LineWidth', 1);

end

