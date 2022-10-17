function [] = drawAirplane(desx, desy, theta)
%DRAWROBOT Draw a robot at a position and rotation angle for a team
%   Draw a robot with position (desx,desy) at rotation angle theta, for
%   either team attack in red, or team defence in blue.

% adjust it so it faces the goal head on
%theta = theta + (pi/2);

% aircraft outline 
x = [0 1 5  7  9 8 11 12 12 11 12 12 11 12 12 11 8 9 7 5 1 0];
y = [6 7 7 12 12 7 7   8  7  7  7  5  5  5  4  5 5 0 0 5 5 6];

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


    plot(sx(1,:), sx(2,:), 'r-', 'LineWidth', 1);


end

