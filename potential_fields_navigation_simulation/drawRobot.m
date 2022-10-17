function [] = drawRobot(desx, desy, theta, team)
%DRAWROBOT Draw a robot at a position and rotation angle for a team
%   Draw a robot with position (desx,desy) at rotation angle theta, for
%   either team attack in red, or team defence in blue.

% adjust it so it faces the goal head on
theta = theta + (pi/2);

% robot outline
x = [1 3 4 6 6 7 7 6 6 5 4 4 3 3 2 1 1 0 0 1 1 3] *0.1;
y = [0 1 1 0 1 1 3 3 4 6 7 8 8 7 6 4 3 3 1 1 0 1] *0.1;

% find the offset from the edge to central point
offstx = (max(x)-min(x))/2;
offsty = (max(y)-min(y))/2;
% put centre of robot at desired location
%offstx = offstx + desx;
%offsty = offsty + desy;
offstx = desx+offstx;
offsty = desy-offsty;
% traslare robot
%x = x+offstx;
%y = y+offsty;

% rotate robot
sx = [x; y];
%theta = 0.5;
for i=1:1:sel(size(x),1,2)
R = [cos(theta) -sin(theta);
     sin(theta) cos(theta)];
sx(:,i) = [offstx offsty]'+(R*sx(:,i));
end

if strcmp(team,'attack')==1
    plot(sx(1,:), sx(2,:), 'r-', 'LineWidth', 3);
end
if strcmp(team,'defence')==1
    plot(sx(1,:), sx(2,:), 'g-', 'LineWidth', 3);
end

end

