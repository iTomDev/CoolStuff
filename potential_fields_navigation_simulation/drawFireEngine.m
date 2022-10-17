function [] = drawCar(desx, desy, theta, team)
%DRAWROBOT Draw a robot at a position and rotation angle for a team
%   Draw a robot with position (desx,desy) at rotation angle theta, for
%   either team attack in red, or team defence in blue.

% adjust it so it faces the goal head on
theta = theta + (pi/2);

x = [0 4 4 3 3 1 1 3 1 1 3 1 1 3 1 1 3 1 1 3 0 0]; 
y = [0 0 8 8 3 3 4 4 4 5 5 5 6 6 6 7 7 7 8 8 8 0];

h = rectangle('Position',[desx desy-1.5 1 1],'Curvature',[1,1],'FaceColor',[0 0 1]);
h = rectangle('Position',[desx desy-3.5 1 1],'Curvature',[1,1],'FaceColor',[1 1 1]);

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

