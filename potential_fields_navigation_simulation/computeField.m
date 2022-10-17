function [field,robotstate2] = computeField(robot1, robot2, obst, goal, role, rn, StaticObs, DynamicObs, robotstate)
%COMPUTEFIELD Compute the potential field at a robot location


% working vars
field = zeros(3,1);


%------------------------------------------------------------------------
%-------------------------   THINK  ---------------------------------------
%------------------------------------------------------------------------

% This computes the logic of the robot so we know which actions to
% implement with the fields. Do this by 'activating' or 'deactivating' a 
% field.
    %1goalid, 2followstatus, 3minimastatus, 4velocitynorm, 5randomfieldx, 6random fieldy, 7randomfieldz, 8randomfieldstatus, 9at goal
% "defines" 
GOALID = 1;
FOLLOWSTATUS = 2;
MINIMASTATUS = 3;
VELOCITYNORM = 4;
RANDOMFIELDX = 5;
RANDOMFIELDY = 6;
RANDOMFIELDZ = 7;
RANDOMFIELDSTATUS = 8;
STANDOFF = 9;
ATGOAL = 10;

%------------------- Local Minima Logic 

RANDOMFIELDSTATUSMAX = 10;
MINIMATHRESHOLD = 0.5;

%{
% check if at minima
if (robotstate(rn,MINIMASTATUS)==0) % not at minima
    % check if at a minima. if velocitynorm is <1
    if (robotstate(rn,VELOCITYNORM)<MINIMATHRESHOLD)
        % increment the waiting time counter
        robotstate(rn,MINIMASTATUS) = robotstate(rn,MINIMASTATUS) + 1;
    end
end

% if at a minima
if (robotstate(rn,MINIMASTATUS)>0) % not at minima
    % if waiting at the minima for a while, generate random field
    if (robotstate(rn,MINIMASTATUS)>5)
        % generate a random field by setting random field decrementer to
        % max
        robotstate(rn,RANDOMFIELDSTATUS) = RANDOMFIELDSTATUSMAX;
        
        % clear minima field
        robotstate(rn,MINIMASTATUS) = 0;
        
    end
end

% if following a random field, reduce the randomfieldstatus each iteration
if (robotstate(rn,RANDOMFIELDSTATUS)>0)
    robotstate(rn,RANDOMFIELDSTATUS) = robotstate(rn,RANDOMFIELDSTATUS) - 1;
end
%}



%------------------- Assign Nearest Goal
% if nearest goal is not set then set it

if (robotstate(rn,GOALID)==0)
% Find nearest goal
    % check that its not assigned to follow another robot
    if (robotstate(rn,FOLLOWSTATUS)==0)
        % loop through goals and find the nearest one
        closest = [inf inf inf];
        for i=1:size(goal,1)
            dist = sqrt((goal(i,2)-robot1(rn,2))^2 + (goal(i,1)-robot1(rn,1))^2);
            % if dist<closest
            if norm(dist) < norm(closest(1:2))
                closest = goal(i,:);
                robotstate(rn,GOALID) = i;
            end
        end
    end
end


%------------------------------------------------------------------------
%-------------------------   ACT  ---------------------------------------
%------------------------------------------------------------------------




if strcmp(role, 'fireengine')==1
if robotstate(rn,ATGOAL)==0   

    %--------- Go to assigned goal
    % compute attractive field for that closest goal
    if (robotstate(rn,FOLLOWSTATUS)==0)
        
        gi = robotstate(rn,GOALID);
        dist = sqrt((goal(gi,2)-robot1(rn,2))^2 + (goal(gi,1)-robot1(rn,1))^2);
        angle = atan2(goal(gi,2)-robot1(rn,2), goal(gi,1)-robot1(rn,1));
        
        % standoff
        %dist = dist - [robotstate(rn,STANDOFF)];
        field = zeros(3,1);
        %abs(dist)
        if abs(dist)>robotstate(rn,STANDOFF)
            % linear field
            k = 0.1;
            goalfield = -k*([cos(angle) 0; 0 sin(angle)]*[dist dist]');
            field = field + [goalfield;0];
            gi = 0;
            % quadratic
            %goalfield = k*([cos(angle) 0; 0 sin(angle)]*[dist^2 dist^2]');
        end
        
        % if at destination
        if abs(dist)<robotstate(rn,STANDOFF)
            robotstate(rn,ATGOAL) = 1;
        end
    end
    % 
    
    %--------- repel from othr vehicle if not following
    dynobstsum = zeros(3,1);
    if robotstate(rn,FOLLOWSTATUS)==0
        % loop through dynamic obstcles
        %numobsts = (sel(size(DynamicObs),1,1));
        numobsts = size(DynamicObs,1);
        for i=1:1:60
            % comput repulsive field for all dynamic obsacles
            % This just obstacles the end points of the lines
            k = 10; %10;
            threshold = 50;%10;
            
            k = 10;
            threshold = 10;
            
            
            dist = sqrt((DynamicObs(i,2)-robot1(rn,2))^2 + (DynamicObs(i,1)-robot1(rn,1))^2);
            angle = atan2(DynamicObs(i,2)-robot1(rn,2), DynamicObs(i,1)-robot1(rn,1));
            if(dist>0)
                % thresholding decreasing quadratic field
                %dynobst = -[cos(angle) 0; 0 sin(angle)]*0.5*k*((1./dist')-(1./[threshold; threshold])).^2;
                %dynobst = k*[sinh(dist-5) 0]';
                dynobst = [cos(angle) 0; 0 sin(angle)]*k* ([1/dist 1/dist]').^3;
                % add the field to the sum
                dynobst;
                dynobstsum = dynobstsum + [dynobst; 0];
                dynobstsum;
            end
        end
    end
    % it to the total field
    field = field + dynobstsum;
    
    goalfield = zeros(2,1);
    %--------- if following, use attractive repulsive field
    if robotstate(rn,FOLLOWSTATUS)>0
        if robotstate(rn,ATGOAL)==0
            fid = robotstate(rn,FOLLOWSTATUS);
            % the goal will be another robot, not the actual "goal"
            % will need a standoff from the robot 
            % standoff is rn*5
            tempgoal = [robot1(fid,1); robot1(fid,2); robot1(fid,3)];
            
            dist = sqrt((tempgoal(2)-robot1(rn,2))^2 + (tempgoal(1)-robot1(rn,1))^2);
            angle = atan2(tempgoal(2)-robot1(rn,2), tempgoal(1)-robot1(rn,1));
            
            if(abs(dist)<abs(robotstate(rn,STANDOFF)))
                robotstate(rn,ATGOAL) = 1;
                
            end
            
            % easy attractive repulsive field
            % linear field at close range
            k = 0.1; %0.1
            %goalfield = -k*([cos(angle) 0; 0 sin(angle)]*([dist;dist]) );
            %
            % works
            k = 5; %5;
            threshold = 3;
            goalfield = ([cos(angle) 0; 0 sin(angle)]*k*( [1/dist; 1/dist] - [1/threshold; 1/threshold] ));
            %
            %threshold = 10;
            %goalfield = ([cos(angle) 0; 0 sin(angle)]*k*( [dist; dist] - [threshold; threshold] ));
            
            k = 0.1; %5;
            threshold = 3;
            goalfield = -([cos(angle) 0; 0 sin(angle)]*k*( [dist; dist]));
        end
    end
    %field
    field = field + [goalfield; 0];
    
    %--------- Local Minima, Use Random Field
    if robotstate(rn,RANDOMFIELDSTATUS) == RANDOMFIELDSTATUSMAX
        % http://planning.cs.uiuc.edu/node225.html
    end
    
    
    % it to the total field
    field = field + dynobstsum;

    % -----------------
    % 2D obstacle

    % meaningfully we may as well just work with the closest point on the
    % obstacle. 
    % static obstacles
    %numobsts = (sel(size(StaticObs),1,1));
    numobsts = size(StaticObs,1);
    obstsum = zeros(3,1);
    
    % form a series of points obstacles between the ends (backup option)
    % it produces a field with a pointsperline of 1 but not with 2
    for ni=1:1:numobsts 
        % find the angle between the two points
        obslinelength = sqrt((StaticObs(ni,1)-StaticObs(ni,4))^2 + (StaticObs(ni,2)-StaticObs(ni,5))^2);
        obsangle = atan2(StaticObs(ni,2)-StaticObs(ni,5), StaticObs(ni,1)-StaticObs(ni,4));
        % find a suitable spacing
        pointsperline = 5;
        obslinespacing = obslinelength/pointsperline;
        % create the points along the line
        Points = zeros(2,pointsperline);
        for j=1:1:pointsperline
            Pointsx = StaticObs(ni,1)-j*obslinespacing*cos(obsangle);
            Pointsy = StaticObs(ni,2)-j*obslinespacing*sin(obsangle);

            k = 2;
            threshold = 2;

            k = 1;
            threshold = 5;
            
            % too strong
            k = 300;
            threshold = 50;
            
            % can get around the angle building but not between houses
            k = 30;
            threshold = 50;
            
            k = 10; %10;
            threshold = 50;%50;
            
            % bad vals-testing
            k = 50; %10;
            threshold = 1; %70;%50;
            
            % upper right
            if((pi/2)>obsangle>=(0))
                % This just obstacles the end points of the lines
                %k = 20;
                %threshold = 2;
                dist = sqrt((Pointsy-robot1(rn,2))^2 + (Pointsx-robot1(rn,1))^2);
                angle = atan2(Pointsy-robot1(rn,2), Pointsx-robot1(rn,1));
                % thresholding decreasing quadratic field
                %repField = -[cos(angle) 0; 0 sin(angle)]*0.5*k*((1./dist')- (1./[threshold; threshold])).^2;
                repField = [cos(angle) 0; 0 sin(angle)]*k* ([1/dist 1/dist]').^3;
                % limit field if its too big
                if abs(repField(1))>10
                    repField(1) = 10;
                end
                if abs(repField(2))>10
                    repField(2) = 10;
                end
                % add the field to the sum
                obstsum = obstsum + [repField; 0];
                %obstsum = obstsum./2;
            end

            % if upper left
            if((pi)>obsangle>=(pi/2))
                %Points(1,j) = -1*Points(1,j);
                Pointsx = -1*Pointsx;
                %
                %k = 20;
                %threshold = 2;
                dist = sqrt((Pointsy-robot1(rn,2))^2 + (Pointsx-robot1(rn,1))^2);
                angle = atan2(Pointsy-robot1(rn,2), Pointsx-robot1(rn,1));
                % thresholding decreasing quadratic field
                %repField = -[cos(angle) 0; 0 sin(angle)]*0.5*k*((1./dist')- (1./[threshold; threshold])).^2;
                repField = [cos(angle) 0; 0 sin(angle)]*k* ([1/dist 1/dist]').^3;
                % limit field if its too big
                if abs(repField(1))>10
                    repField(1) = 10;
                end
                if abs(repField(2))>10
                    repField(2) = 10;
                end
                % add the field to the sum
                obstsum = obstsum + [repField; 0];
                %obstsum = obstsum./2;
            end
            % lower left
            if(((3*pi)/2)>obsangle>=(pi))
                %Points(1,j) = -1*Points(1,j);
                %Points(2,j) = -1*Points(2,j);
                Pointsx = -1*Pointsx;
                Pointsy = -1*Pointsy;
                % This just obstacles the end points of the lines
                %k = 20;
                %threshold = 2;
                dist = sqrt((Pointsy-robot1(rn,2))^2 + (Pointsx-robot1(rn,1))^2);
                angle = atan2(Pointsy-robot1(rn,2), Pointsx-robot1(rn,1));
                % thresholding decreasing quadratic field
                %repField = -[cos(angle) 0; 0 sin(angle)]*0.5*k*((1./dist')- (1./[threshold; threshold])).^2;
                repField = [cos(angle) 0; 0 sin(angle)]*k* ([1/dist 1/dist]').^3;
                % limit field if its too big
                if abs(repField(1))>10
                    repField(1) = 10;
                end
                if abs(repField(2))>10
                    repField(2) = 10;
                end
                % add the field to the sum
                obstsum = obstsum + [repField; 0];
                %obstsum = obstsum./2;
            end
            % lower right
            if((2*pi)>obsangle>=(3*pi/2))
                %Points(2,j) = -1*Points(2,j);
                Pointsy = -1*Pointsy;
                % This just obstacles the end points of the lines
                %k = 20;
                %threshold = 2;
                dist = sqrt((Pointsy-robot1(rn,2))^2 + (Pointsx-robot1(rn,1))^2);
                angle = atan2(Pointsy-robot1(rn,2), Pointsx-robot1(rn,1));
                % thresholding decreasing quadratic field
                %repField = -[cos(angle) 0; 0 sin(angle)]*0.5*k*((1./dist')- (1./[threshold; threshold])).^2;
                repField = [cos(angle) 0; 0 sin(angle)]*k* ([1/dist 1/dist]').^3;
                % limit field if its too big
                if abs(repField(1))>10
                    repField(1) = 10;
                end
                if abs(repField(2))>10
                    repField(2) = 10;
                end
                % add the field to the sum
                obstsum = obstsum + [repField; 0];
                %obstsum = obstsum./2;
            end
        end % ni obs
        obstsum;
    end
    % add final global obstacle to total field
    field = field + obstsum;
    field;
    
    % complex long range goal, close range obstacle - aircraft
    
    % other fire engines are obstacles to avoid, lightly
    
    % just to be sure - check if at goal
    %if it has a goal
    if robotstate(rn,GOALID) >0
        dist = sqrt((goal(robotstate(rn,GOALID),2)-robot1(rn,2))^2 + (goal(robotstate(rn,GOALID),1)-robot1(rn,1))^2);
        % close enough
        if(abs(dist)<robotstate(rn,STANDOFF))
            robotstate(rn,ATGOAL) = 1;
        end
    end
    
end
end
    
field;    
    


%{
    % local minima. Use random field to help escape. 
    % if velocity is very low, increment integrator
    if norm(field)<10
       robotintegrator(1) = robotintegrator(1) +1; 
    end
    % if velocity continues to be low, compute a random field and add it on
    if robotintegrator(1)>20
        % random field - how to keep the same field?
        
        % decrement integrator
        robotintegrator(1) = robotintegrator(1) -1;
    end
    % 
   %}

% update current velocity - for detecting minima 
robotstate(rn,VELOCITYNORM) = norm(field);

robotstate2 = robotstate;



% -------------------------------------------------------------------------
%                   PLOT STATIC OBSTACLE FIELD
% -------------------------------------------------------------------------
if strcmp(role, 'plotstaticobstaclefield')==1
    %
    field = zeros(3,1);
    % -----------------
    % 2D obstacle
    
    % meaningfully we may as well just work with the closest point on the
    % obstacle. 
    % static obstacles
    %numobsts = (sel(size(StaticObs),1,1));
    numobsts = size(StaticObs,1);
    obstsum = zeros(3,1);
    %for i=1:1:2;%  numobsts
    
    % form a series of points obstacles between the ends (backup option)
    % it produces a field with a pointsperline of 1 but not with 2
    for ni=1:1:numobsts 
        % find the angle between the two points
        obslinelength = sqrt((StaticObs(ni,1)-StaticObs(ni,4))^2 + (StaticObs(ni,2)-StaticObs(ni,5))^2);
        obsangle = atan2(StaticObs(ni,2)-StaticObs(ni,5), StaticObs(ni,1)-StaticObs(ni,4));
        % find a suitable spacing
        pointsperline = 5;
        obslinespacing = obslinelength/pointsperline;
        % create the points along the line
        Points = zeros(2,pointsperline);
        for j=1:1:pointsperline 
            Pointsx = StaticObs(ni,1)-j*obslinespacing*cos(obsangle);
            Pointsy = StaticObs(ni,2)-j*obslinespacing*sin(obsangle);
            % bad vals-testing
            k = 50; %10;
            threshold = 1; %70;%50;
            
            % upper right
            if((pi/2)>obsangle>=(0))
                % This just obstacles the end points of the lines
                %k = 20;
                %threshold = 2;
                dist = sqrt((Pointsy-robot1(rn,2))^2 + (Pointsx-robot1(rn,1))^2);
                %if dist>0 
                    angle = atan2(Pointsy-robot1(rn,2), Pointsx-robot1(rn,1));
                    % compute field
                    repField = [cos(angle) 0; 0 sin(angle)]*k* ([1/dist 1/dist]').^3;
                    % limit field if its too big
                    if abs(repField(1))>10
                        repField(1) = 10;
                    end
                    if abs(repField(2))>10
                        repField(2) = 10;
                    end
                    % add the field to the sum
                    obstsum = obstsum + [repField; 0];
                    %obstsum = obstsum./2;
                %end
            end

            % if upper left
            if((pi)>obsangle>=(pi/2))
                %Points(1,j) = -1*Points(1,j);
                Pointsx = -1*Pointsx;
                %
                %k = 20;
                %threshold = 2;
                dist = sqrt((Pointsy-robot1(rn,2))^2 + (Pointsx-robot1(rn,1))^2);
                %if dist>0 
                    angle = atan2(Pointsy-robot1(rn,2), Pointsx-robot1(rn,1));
                    % thresholding decreasing quadratic field
                    %repField = -[cos(angle) 0; 0 sin(angle)]*0.5*k*((1./dist')- (1./[threshold; threshold])).^2;
                    repField = [cos(angle) 0; 0 sin(angle)]*k* ([1/dist 1/dist]').^3;
                    % limit field if its too big
                    if abs(repField(1))>10
                        repField(1) = 10;
                    end
                    if abs(repField(2))>10
                        repField(2) = 10;
                    end
                    % add the field to the sum
                    obstsum = obstsum + [repField; 0];
                    %obstsum = obstsum./2;
                %end
            end
            % lower left
            if(((3*pi)/2)>obsangle>=(pi))
                %Points(1,j) = -1*Points(1,j);
                %Points(2,j) = -1*Points(2,j);
                Pointsx = -1*Pointsx;
                Pointsy = -1*Pointsy;
                % This just obstacles the end points of the lines
                %k = 20;
                %threshold = 2;
                dist = sqrt((Pointsy-robot1(rn,2))^2 + (Pointsx-robot1(rn,1))^2);
                %if dist>0 
                    angle = atan2(Pointsy-robot1(rn,2), Pointsx-robot1(rn,1));
                    % thresholding decreasing quadratic field
                    %repField = -[cos(angle) 0; 0 sin(angle)]*0.5*k*((1./dist')- (1./[threshold; threshold])).^2;
                    repField = [cos(angle) 0; 0 sin(angle)]*k* ([1/dist 1/dist]').^3;
                    % limit field if its too big
                    if abs(repField(1))>10
                        repField(1) = 10;
                    end
                    if abs(repField(2))>10
                        repField(2) = 10;
                    end
                    % add the field to the sum
                    obstsum = obstsum + [repField; 0];
                    %obstsum = obstsum./2;
                %end
            end
            % lower right
            if((2*pi)>obsangle>=(3*pi/2))
                %Points(2,j) = -1*Points(2,j);
                Pointsy = -1*Pointsy;
                % This just obstacles the end points of the lines
                %k = 20;
                %threshold = 2;
                dist = sqrt((Pointsy-robot1(rn,2))^2 + (Pointsx-robot1(rn,1))^2);
                %if dist>0 
                    angle = atan2(Pointsy-robot1(rn,2), Pointsx-robot1(rn,1));
                    % thresholding decreasing quadratic field
                    %repField = -[cos(angle) 0; 0 sin(angle)]*0.5*k*((1./dist')- (1./[threshold; threshold])).^2;
                    repField = [cos(angle) 0; 0 sin(angle)]*k* ([1/dist 1/dist]').^3;
                    % limit field if its too big
                    if abs(repField(1))>10
                        repField(1) = 10;
                    end
                    if abs(repField(2))>10
                        repField(2) = 10;
                    end
                    % add the field to the sum
                    obstsum = obstsum + [repField; 0];
                    %obstsum = obstsum./2;
                %end
            end 
        end % ni obs
    end
    %obstsum % ------------------------------------------------------------------
    % add final global obstacle to total field
    field = field + obstsum;
end
 
    
%%
% quadratic attractive field
%0.5*k*(goal-robot)^2
% this is combined with a conical field to limit the effect of the
% quadratic term at large distances. 
% 
