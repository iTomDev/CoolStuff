function [pos] = getPosition(pos, encoderprev, encodernow)
% convert encoder increment into position
% Thomas Pile, 21048743
% References
% https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf
% https://www.cyberbotics.com/doc/guide/using-the-e-puck-robot

% constants
% experimentally 7.9538 "pos" = 1mm in a straight line
% (1751pos-717pos)/130mm = 7.9538pos/mm

% for testing
% encoderprev = [0 0];
% encodernow = [1034 1034];
% pos = [0,0,0];

b = 52; % wheelbase, [mm]

% encoderprev = [0 0];
% encodernow = [0 0];

% convert encoder to mm
encoderprev = encoderprev/7.9538;
encodernow = encodernow/7.9538;

% series = zeros(10,3);
% for i=1:1:10
    % find changes in the encoder positions
    dsl = encodernow(1)-encoderprev(1);
    dsr = encodernow(2)-encoderprev(2);
    % ds = average change in encoders
    ds = (dsr+dsl)/2;
    %
    ds2 = (dsr-dsl)/2*b;
    ds3 = (dsr-dsl)/b;
    
    % find deltax and deltay = change in x and y position
    theta = pos(3);
    %theta = (dsr-dsl)/b;
    dp = zeros(3,1);
    dp = [ds*cos(theta+ds3);
          ds*sin(theta+ds3);
          ds3];
    pos = pos + dp';
    pos
%     series(i,:) = pos;
    
%     if  3<i<5
%         encoderprev = encodernow;
%         encodernow(1) = encodernow(1) + 20;
%         encodernow(2) = encodernow(2) + 10;
%     else
%         encoderprev = encodernow;
%         encodernow(1) = encodernow(1) + 10;
%         encodernow(2) = encodernow(2) + 10;
%     end
%end

% display map at the end
% for i=1:1:9
%     drawLine(series(i,1),series(i,2),series(i+1,1),series(i+1,2),'blue')
% end
% axis([-300 300 -300 300])

end

