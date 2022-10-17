% My code
% I want dynamic obstacles so I will have to compute the field again each
% time the robot or the obstacles move.
% I have to use my own code really as I want multiple obstacles and
% multiple robots which the provided code doesn't really allow for. Check
% withg Lyuba that this ok... 
d2r = 0.0174533;


close all;

%%

clf;
clc;
clear;

szGridX = 100;
szGridY = 100;

% draw environment
figure(1)
title('Environment')
hold on
axis([0 szGridX 0 szGridY]);
grid on;
% 

% border left
BorderLx1 = [000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000];
BorderLy1 = [000 005 010 015 020 025 030 035 040 045 050 055 060 065 070 075 080 085 090 095 100];
BorderLx2 = 0.5*[001 001 001 001 001 001 001 001 001 001 001 001 001 001 001 001 001 001 001 001 001 000];
BorderLy2 = [100 095 090 085 080 075 070 065 060 055 050 045 040 035 030 025 020 015 010 005 000 000];
BorderLx = [BorderLx1 BorderLx2];
BorderLy = [BorderLy1 BorderLy2];
hgBorderL = hgtransform;
gfxBorderL = patch('XData',BorderLx,'YData',BorderLy,'FaceColor','white','Parent',hgBorderL);

% border lower
BorderBx1 = [000 005 010 015 020 025 030 035 040 045 050 055 060 065 070 075 080 085 090 095 100];
BorderBy1 = [000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000];
BorderBx2 = [100 095 090 085 080 075 070 065 060 055 050 045 040 035 030 025 020 015 010 005 000 000];
BorderBy2 = [001 001 001 001 001 001 001 001 001 001 001 001 001 001 001 001 001 001 001 001 001 000];
BorderBx = [BorderBx1 BorderBx2];
BorderBy = [BorderBy1 BorderBy2];
hgBorderB = hgtransform;
gfxBorderB = patch('XData',BorderBx,'YData',BorderBy,'FaceColor','white','Parent',hgBorderB);

% border top
BorderTx1 = [000 005 010 015 020 025 030 035 040 045 050 055 060 065 070 075 080 085 090 095 100];
BorderTy1 = [099 099 099 099 099 099 099 099 099 099 099 099 099 099 099 099 099 099 099 099 099];
BorderTx2 = [100 095 090 085 080 075 070 065 060 055 050 045 040 035 030 025 020 015 010 005 000 000];
BorderTy2 = [100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 099];
BorderTx = [BorderTx1 BorderTx2];
BorderTy = [BorderTy1 BorderTy2];
hgBorderT = hgtransform;
gfxBorderT = patch('XData',BorderTx,'YData',BorderTy,'FaceColor','white','Parent',hgBorderT);

% border right
BorderRx1 = [099 099 099 099 099 099 099 099 099 099 099 099 099 099 099 099 099 099 099 099 099];
BorderRy1 = [000 005 010 015 020 025 030 035 040 045 050 055 060 065 070 075 080 085 090 095 100];
BorderRx2 = [100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 099];
BorderRy2 = [100 095 090 085 080 075 070 065 060 055 050 045 040 035 030 025 020 015 010 005 000 000];
BorderRx = [BorderRx1 BorderRx2];
BorderRy = [BorderRy1 BorderRy2];
hgBorderR = hgtransform;
gfxBorderR = patch('XData',BorderRx,'YData',BorderRy,'FaceColor','white','Parent',hgBorderR);


%{
Borderx1 = [00 05 10 15 20 25 30 35 40 45 50 55 60 65 70 75 80 85 90 95 100];
Bordery1 = [00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 000]; 
Borderx2 = [100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100];
Bordery2 = [005 010 015 020 025 030 035 040 045 050 055 060 065 070 075 080 085 090 095 100];
Borderx3 = [095 090 085 080 075 070 065 060 055 050 045 040 035 030 025 020 015 010 005 000];
Bordery3 = [100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100];
Borderx4 = [000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000];
Bordery4 = [095 090 085 080 075 070 065 060 055 050 045 040 035 030 025 020 015 010 005 000];
Borderx = [Borderx1 Borderx2 Borderx3 Borderx4];
Bordery = [Bordery1 Bordery2 Bordery3 Bordery4];
%}

%{
% draw long building - long
%drawLongBuilding(10,70);
hgLongBuilding = hgtransform;
%LongBuildingx = [0 00 10 20 30 40 50 50 40 30 20 10 00];
%LongBuildingy = [0 10 10 10 10 10 10 00 00 00 00 00 00];
LongBuildingx = [0 00 00 05 10 15 20 25 30 35 40 45 50 50 50 45 40 35 30 25 20 15 10 05 00];
LongBuildingy = [0 05 10 10 10 10 10 10 10 10 10 10 10 05 00 00 00 00 00 00 00 00 00 00 00];
gfxLongBuilding = patch('XData',LongBuildingx,'YData',LongBuildingy,'FaceColor','white','Parent',hgLongBuilding);
hgLongBuilding.Matrix = makehgtform('translate',[-10,-10,0]);
%}

% draw river
%drawRiver(0,0);
hgRiver1 = hgtransform;
hgRiver2 = hgtransform;
hgRiver3 = hgtransform;
river1x = [0 0 12 24 25 25 18 14 0];
river1y = [0 3  3  5  5 2  1  0 0];
%river2x = [40 48 52 64 80 80 68 62 56 54 48 40];
%river2y = [5  04 04 03 04 01 00 00 01 01 01 02];
river2x = [40 44 48 52 57 64 70 75 80 80 75 70 68 64 62 56 54 48 44 40];
river2y = [05 04 04 04 03 03 04 04 04 01 01 01 00 00 00 01 01 01 02 02];
%river3x = [90 95 100 100 95 90];
%river3y = [4   4   4   1  1  1];
river3x = [95 100 100 95];
river3y = [ 4   4   1  1];
gfxRiver1 = patch('XData',river1x,'YData',river1y,'FaceColor','blue','Parent',hgRiver1);
gfxRiver2 = patch('XData',river2x,'YData',river2y,'FaceColor','blue','Parent',hgRiver2);
gfxRiver3 = patch('XData',river3x,'YData',river3y,'FaceColor','blue','Parent',hgRiver3);
hgRiver1.Matrix = makehgtform('translate',[0,30,0]);
hgRiver2.Matrix = makehgtform('translate',[0,30,0]);
hgRiver3.Matrix = makehgtform('translate',[0,30,0]);

% runway and bridge
% This is a massive pain to draw while maintaing compatibility with older
% versions of Matlab, AND being able to keep the function parameters for
% generating the field clean! 
%runway1x = [90 90 95 95 20  20 30 30];
%runway1y = [38 25 25  5  5  25 25 42];
%runway2x = [40 40 80 80];
%runway2y = [42 25 25 38];
% line 1
hgRunway1 = hgtransform;
%Runway1x = [90 90 90 90 90];
%Runway1y = [38 35 32 28 25];
Runway1x = [95 95 95 95 95];
Runway1y = [38 35 32 28 25];
gfxRunway1 = patch('XData',Runway1x,'YData',Runway1y,'FaceColor','red','Parent',hgRunway1);
% line 2
hgRunway2 = hgtransform;
Runway2x = [95 95];
Runway2y = [25 25];
gfxRunway2 = patch('XData',Runway2x,'YData',Runway2y,'FaceColor','red','Parent',hgRunway2);
% line 3
hgRunway3 = hgtransform;
Runway3x = [95 95 95 95 95];
Runway3y = [25 20 15 10 05];
gfxRunway3 = patch('XData',Runway3x,'YData',Runway3y,'FaceColor','red','Parent',hgRunway3);
% line 4 long
hgRunway4 = hgtransform;
Runway4x = [95 90 85 80 75 70 65 60 55 50 45 40 35 30 25 20];
Runway4y = [05 05 05 05 05 05 05 05 05 05 05 05 05 05 05 05];
gfxRunway4 = patch('XData',Runway4x,'YData',Runway4y,'FaceColor','red','Parent',hgRunway4);
% line 5 long
hgRunway5 = hgtransform;
Runway5x = [20 20 20 20 20];
Runway5y = [05 10 15 20 25];
gfxRunway5 = patch('XData',Runway5x,'YData',Runway5y,'FaceColor','red','Parent',hgRunway5);
% line 6
hgRunway6 = hgtransform;
Runway6x = [20 25];
Runway6y = [25 25];
gfxRunway6 = patch('XData',Runway6x,'YData',Runway6y,'FaceColor','red','Parent',hgRunway6);
% line 7 long
hgRunway7 = hgtransform;
Runway7x = [25 25 25 25 25];
Runway7y = [25 30 35 40 42];
gfxRunway7 = patch('XData',Runway7x,'YData',Runway7y,'FaceColor','red','Parent',hgRunway7);
% line 8
hgRunway8 = hgtransform;
Runway8x = [40 40 40 40 40];
Runway8y = [42 40 35 30 25];
gfxRunway8 = patch('XData',Runway8x,'YData',Runway8y,'FaceColor','red','Parent',hgRunway8);
% line 9 long
hgRunway9 = hgtransform;
Runway9x = [40 45 50 55 60 65 70 75 80];
Runway9y = [25 25 25 25 25 25 25 25 25];
gfxRunway9 = patch('XData',Runway9x,'YData',Runway9y,'FaceColor','red','Parent',hgRunway9);
% line 10
hgRunway10 = hgtransform;
Runway10x = [80 80 80 80];
Runway10y = [25 30 35 38];
gfxRunway10 = patch('XData',Runway10x,'YData',Runway10y,'FaceColor','red','Parent',hgRunway10);

% fire station 1
%drawFirestation(20,90);
hgFirestation1 = hgtransform;
%Firestationx = [0  0 20 20 19 19 01 01 00 ];
%Firestationy = [0 10 10  0  0 09 09 00 00 ];
Firestationx = [0  0 20 20 19 19 01 01 00 ] + 20;
Firestationy = [0 10 10  0  0 09 09 00 00 ] + 90;
gfxFirestation1 = patch('XData',Firestationx,'YData',Firestationy,'FaceColor','red','Parent',hgFirestation1);
%hgFirestation1.Matrix = makehgtform('translate',[20,90,0]);

% fire station 2
%drawFirestation(60,90);
hgFirestation2 = hgtransform;
Firestationx = [0  0 20 20 19 19 01 01 00 ];
Firestationy = [0 10 10  0  0 09 09 00 00 ];
gfxFirestation2 = patch('XData',Firestationx,'YData',Firestationy,'FaceColor','red','Parent',hgFirestation2);
hgFirestation2.Matrix = makehgtform('translate',[60,90,0]);

% draw house 1
hgHouse1 = hgtransform;
%Housex = [10 10 20 25 25 10];
%Housey = [50 80 80 75 50 50];
Housex = [10 10 20 25 25 10];
Housey = [50 80 80 75 50 50];
gfxHouse1 = patch('XData',Housex,'YData',Housey,'FaceColor','white','Parent',hgHouse1);
%hgHouse1.Matrix = makehgtform('translate',[5,50,0]);

% draw house 2
hgHouse2 = hgtransform;
%Housex = [40 40 50 70 75 75 40];
%Housey = [40 50 50 70 70 40 40];
Housex = [40 40 50 55 60 65 70 75 75 75 75 75 75 70 65 60 55 50 45 40];
Housey = [40 50 50 55 60 65 70 70 65 60 55 50 40 40 40 40 40 40 40 40];
gfxHouse2 = patch('XData',Housex,'YData',Housey,'FaceColor','white','Parent',hgHouse2);
%hgHouse2.Matrix = makehgtform('translate',[0,10,0]);

% draw house 3
hgHouse3 = hgtransform;
Housex = [40 40 55 55 45 40];
Housey = [65 80 80 75 65 65];
gfxHouse3 = patch('XData',Housex,'YData',Housey,'FaceColor','white','Parent',hgHouse3);
%hgHouse3.Matrix = makehgtform('translate',[60,50,0]);

% draw house 4
% drawHouse(70,70);
hgHouse4 = hgtransform;
Housex = [90 90 99 99 90];
Housey = [55 80 80 55 55];
gfxHouse4 = patch('XData',Housex,'YData',Housey,'FaceColor','white','Parent',hgHouse4);
%hgHouse4.Matrix = makehgtform('translate',[85,50,0]);

% draw static tower
%drawTower(5,20);
hgTower1 = hgtransform;
Tower1x = [0 0 2 3 5 5 3 2 0];
Tower1y = [2 3 5 5 3 2 0 0 2];
gfxTower1 = patch('XData',Tower1x,'YData',Tower1y,'FaceColor','red','Parent',hgTower1);
hgTower1.Matrix = makehgtform('translate',[5,20,0]);

% draw static aircraft
%drawAirplane(15,25,3.5);
hgStaticAircraft1 = hgtransform;
%x = [0 1 5  7  9 8 11 12 12 11 12 12 11 12 12 11 8 9 7 5 1 0];
%y = [6 7 7 12 12 7 7   8  7  7  7  5  5  5  4  5 5 0 0 5 5 6];
x = [0 1 5  7  9 8 11 12 12 12 12 11 8 9 7 5 1 0];
y = [6 7 7 12 12 7 7   8  7  5  4  5 5 0 0 5 5 6];
gfxStaticAircraft1 = patch('XData',x,'YData',y,'FaceColor','red','Parent',hgStaticAircraft1);
hgStaticAircraft1.Matrix = makehgtform('translate',[15,15,3.5])*makehgtform('zrotate',3);

% draw crash aeroplane
%drawAirplane(50,50,pi/9)
hgCrashAircraft = hgtransform;
x = [0 1 5  7  9 8 11 12 12 12 12 11 8 9 7 5 1 0];
y = [6 7 7 12 12 7 7   8  7  5  4  5 5 0 0 5 5 6];
gfxCrashAircraft = patch('XData',x,'YData',y,'FaceColor','red','Parent',hgCrashAircraft);
hgCrashAircraft.Matrix = makehgtform('translate',[100,8,0]);

% draw generic flying aeroplane 1
% draw generic flying aeroplane 2

% draw ordinary car 1
% draw ordinary car 2

% fire engine 1
%drawFireEngine(48, 48, 0, 'defence');
hgFireEngine1 = hgtransform;
FireEnginex = [0 4 4 3 3 1 1 3 1 1 3 1 1 3 1 1 3 1 1 3 0 0]; 
FireEnginey = [0 0 8 8 3 3 4 4 4 5 5 5 6 6 6 7 7 7 8 8 8 0];
gfxFireEngine1 = patch('XData',FireEnginex,'YData',FireEnginey,'FaceColor','red','Parent',hgFireEngine1);
hgFireEngine1.Matrix = makehgtform('translate',[23,90.5,0]);

% fire engine 2
%drawFireEngine(48, 48, 0, 'defence');
hgFireEngine2 = hgtransform;
FireEnginex = [0 4 4 3 3 1 1 3 1 1 3 1 1 3 1 1 3 1 1 3 0 0]; 
FireEnginey = [0 0 8 8 3 3 4 4 4 5 5 5 6 6 6 7 7 7 8 8 8 0];
gfxFireEngine2 = patch('XData',FireEnginex,'YData',FireEnginey,'FaceColor','red','Parent',hgFireEngine2);
hgFireEngine2.Matrix = makehgtform('translate',[32,90.5,0]);

% fire engine 3
%drawFireEngine(48, 48, 0, 'defence');
hgFireEngine3 = hgtransform;
FireEnginex = [0 4 4 3 3 1 1 3 1 1 3 1 1 3 1 1 3 1 1 3 0 0]; 
FireEnginey = [0 0 8 8 3 3 4 4 4 5 5 5 6 6 6 7 7 7 8 8 8 0];
gfxFireEngine3 = patch('XData',FireEnginex,'YData',FireEnginey,'FaceColor','red','Parent',hgFireEngine3);
hgFireEngine3.Matrix = makehgtform('translate',[63,90.5,0]);

% fire engine 4
%drawFireEngine(48, 48, 0, 'defence');
hgFireEngine4 = hgtransform;
FireEnginex = [0 4 4 3 3 1 1 3 1 1 3 1 1 3 1 1 3 1 1 3 0 0]; 
FireEnginey = [0 0 8 8 3 3 4 4 4 5 5 5 6 6 6 7 7 7 8 8 8 0];
gfxFireEngine4 = patch('XData',FireEnginex,'YData',FireEnginey,'FaceColor','red','Parent',hgFireEngine4);
hgFireEngine4.Matrix = makehgtform('translate',[90,45,0]);


axis([0 szGridX 0 szGridY]);
daspect([1 1 1]) % lock the aspect ratiom to drawings look good
%
% set up variables etc

% create a array of arrays of static pints which cant be put in graphics
% objects without having to use the 2017 version of matlab -_-
% cant even do that without making it a mess -_- y me?

% create an array of all static obstacle transforms
% hg means hgtransform
hgStaticObs = ...
    [hgBorderL
    hgBorderR
    hgBorderT
    hgBorderB
    hgRiver1
    hgRiver2
    hgRiver3
    hgRunway1
    hgRunway2
    hgRunway3
    hgRunway4
    hgRunway5
    hgRunway6
    hgRunway7
    hgRunway8
    hgRunway9
    hgRunway10
    hgFirestation1
    hgFirestation2
    hgHouse1
    hgHouse2
    hgHouse3
    hgHouse4
    hgTower1
    hgStaticAircraft1
    ];
% and create an array of static obstacle image obstacles 
% gfx means graphics
gfxStaticObs = ...
    [gfxBorderL
    gfxBorderR
    gfxBorderT
    gfxBorderB
    gfxRiver1
    gfxRiver2
    gfxRiver3
    gfxRunway1
    gfxRunway2
    gfxRunway3
    gfxRunway4
    gfxRunway5
    gfxRunway6
    gfxRunway7
    gfxRunway8
    gfxRunway9
    gfxRunway10
    gfxFirestation1
    gfxFirestation2
    gfxHouse1
    gfxHouse2
    gfxHouse3
    gfxHouse4
    gfxTower1
    gfxStaticAircraft1
    ];
% create an array of all static obstacles with the transform applied to the
% location coords. This is way more efficient as we only need to compute
% these once (they are static obstacles)
% tfx means transformed
tfxStaticObs = getAllTransformedVectorsAsArray(gfxStaticObs,hgStaticObs);
% null out any z component vals
%for i=1:1:sel(size(tfxStaticObs),1,1)
for i=1:1:size(tfxStaticObs,1)
    if tfxStaticObs(i,3)>0
        tfxStaticObs(i,3) = 0;
    end
    if tfxStaticObs(i,6)>0
        tfxStaticObs(i,6) = 0;
    end
end

% create an array of all dynamic obstacle transform objects
hgDynamicObs = ...
    [hgCrashAircraft
    hgFireEngine1
    hgFireEngine2
    hgFireEngine3
    hgFireEngine4];
% and create an array of dynamic obstacle image obstacles 
gfxDynamicObs = ...
    [gfxCrashAircraft
    gfxFireEngine1
    gfxFireEngine2
    gfxFireEngine3
    gfxFireEngine4];
% create an array of all static obstacles with the transform applied to the
% location coords. This needs recomputing everytime you use it. Its just
% here for completeness so you can find it!
tfxDynamicObs = getAllTransformedVectorsAsArray(gfxDynamicObs,hgDynamicObs);

% draw navigation
goal = ...
    [40,15,0; % static goal (parking location)
     100,8,0; % dynamic goal (aircraft)
      80 10,0]; % right side of runway
robot = ... % robot locations, own team. cols: x, y, z. defenders
    [26 89 0;
     35 89 0;
     66 89 0;
     90 45 0];
obst = ...  % all obstacle. cols: x,y,z,weight
    [50, 5, 0];

% robot state
% used to implement the behavours
% followstatus: 0 not use, >0 id of DynamicObs being followed
% minimastatus: 0 not at local minima, >0 time waiting at minima (delay before getting random field)
% randomfield x,y,z: the random direction issues by random
% at goal. 0 not at goal, 1 at goal 
robotstate = ...
    [     0,            0,            0,            0,            0,             0,            0,                 0,       5,      0;  % fire engine 1
          0,            1,            0,            0,            0,             0,            0,                 0,      10,      0;  % fire engine 2
          2,            0,            0,            0,            0,             0,            0,                 0,      10,      0;  % fire engine 3
          3,            0,            0,            0,            0,             0,            0,                 0,       5,      0]; % fire engine 4
  0;%goalid, followstatus, minimastatus, velocitynorm, randomfieldx, random fieldy, randomfieldz, randomfieldstatus, standoff, atgoal
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
  
%%
% plot static field 
% warning, this is slow! ~4 mins

[x y] = meshgrid(1:1:101,1:1:101); 
% size of the field
szGridx = size(x,1);
szGridy = size(x,2);

fieldx = zeros(szGridx,szGridy);
fieldy = zeros(szGridx,szGridy);

% loop through the field
for m=1:1:szGridx;
    for n=1:1:szGridy;
        testlocs = [n,m,0];
        [field,robotstateret] = computeField(testlocs, robot, obst, goal, 'plotstaticobstaclefield',1, tfxStaticObs, tfxDynamicObs, robotstate);
        fieldx(m,n) = field(1);
        fieldy(m,n) = field(2);
        
    end
end

% draw static field
figure(1)
quiver(x,y,-fieldx,-fieldy);
figure(2)
fieldmag = sqrt(fieldx.^2 + fieldy.^2)
surf(fieldmag)
surf(fieldx,fieldy)
hold on
daspect([1 1 1]) % lock the aspect ratio to make drawings look good

%%

% place a circle at just the endpoints of the obstacle lines
for i=1:1:size(tfxStaticObs,1)
    drawCircle(tfxStaticObs(i,1), tfxStaticObs(i,2), 1);
    drawCircle(tfxStaticObs(i,4), tfxStaticObs(i,5), 1);
    axis([0 100 0 100]);
end

% place a circle at each point interpolating between the ends of
for i=1:1:size(tfxStaticObs,1)
    obslinelength = sqrt((tfxStaticObs(i,1)-tfxStaticObs(i,4))^2 + (tfxStaticObs(i,2)-tfxStaticObs(i,5))^2);
    obsangle = atan2(tfxStaticObs(i,2)-tfxStaticObs(i,5), tfxStaticObs(i,1)-tfxStaticObs(i,4));
    % find a suitable spacing
    pointsperline = 5;
    obslinespacing = obslinelength/pointsperline;
    % create the points along the line
    Points = zeros(2,pointsperline);
    for j=1:1:pointsperline
        Points(1,j) = tfxStaticObs(i,1)-j*obslinespacing*cos(obsangle);
        Points(2,j) = tfxStaticObs(i,2)-j*obslinespacing*sin(obsangle);
        % upper right
        if((pi/2)>obsangle>=(0))
        end
        % if upper left
        if((pi)>obsangle>=(pi/2))
            Points(1,j) = -1*Points(1,j);
        end
        % lower left
        if(((3*pi)/2)>obsangle>=(pi))
            Points(1,j) = -1*Points(1,j);
            Points(2,j) = -1*Points(2,j);
        end
        % lower right
        if((2*pi)>obsangle>=(3*pi/2))
            Points(2,j) = -1*Points(2,j);
        end

    end
    for j=1:1:pointsperline
        drawCircle(Points(1,j), Points(2,j), .5);
    end
    % endpoints
    drawCircle(tfxStaticObs(i,1), tfxStaticObs(i,2), .5);
    drawCircle(tfxStaticObs(i,4), tfxStaticObs(i,5), .5);
    axis([0 100 0 100]);
end
    
%%
% moving the robots through the airport scenario

t = 0;
tfinal = 500;   
deltat = 0.2;
GOALTHRESHOLD = 5;
robot2goalcounter = 0;

% loop until at the objective
d2g =  sqrt( (robot(1,1)-goal(1,1))^2 + (robot(1,2)-goal(1,2))^2 );
while(t<tfinal)
    while 2;%(d2g>GOALTHRESHOLD)
        
        % create an array of all static obstacles with the transform applied to the
        % location coords. This needs recomputing everytime you use it.
        tfxDynamicObs = getAllTransformedVectorsAsArray(gfxDynamicObs,hgDynamicObs);
        
        % crash aeroplane sequence
        if(t<10)
            angle = 0;
            hgCrashAircraft.Matrix = makehgtform('translate',[100-6*t,8,0]);
            % update new position of aircraft(goal)
            goal(2,:) = tfxDynamicObs(1,1:3)';
        end
        
        % fire engine movement
        for i=1:size(robot,1)
            active = 1;
            
            % delay robot2 for a few secs
            if t<2 && i==2
                active = 0;
            end
            % delay robot4 for a few secs
            if t<5.5 && i==4
                active = 0;
            end
            % robots at goal are inactive
            if robotstate(i,10)>0
                active = 0;
            end
            
            % second task for robot 2
            rtsk = 2;
            if i==rtsk % only the appropriate robot
                if robotstate(rtsk,ATGOAL)==1
                    % completed task 1
                    if(robot2goalcounter==0)
                        % get ready for second scenario
                        robotstate(rtsk,ATGOAL)=0;
                        active = 1;
                        % copy old goal array to a new bigger one
                        szGoal = size(goal,1);
                        goal2 = goal;
                        goal = zeros(szGoal+1,3);
                        goal(1:szGoal,:) = goal2(:,:);
                        % add a goal to the goals array
                        goal(szGoal+1,:) = [30,55,0];
                        % and set that as the goal
                        robotstate(rtsk,GOALID) = szGoal+1;
                        robotstate(rtsk,FOLLOWSTATUS) = 0;
                    end % task1
                    % completed task 1
                    if(robot2goalcounter==1)
                        % get ready for second scenario
                        robotstate(rtsk,ATGOAL)=0;
                        active = 1;
                        % copy old goal array to a new bigger one
                        szGoal = size(goal,1);
                        goal2 = goal;
                        goal = zeros(szGoal+1,3);
                        goal(1:szGoal,:) = goal2(:,:);
                        % add a goal to the goals array
                        goal(szGoal+1,:) = [65,80,0];
                        % and set that as the goal
                        robotstate(rtsk,GOALID) = szGoal+1;
                        robotstate(rtsk,FOLLOWSTATUS) = 0;
                    end % task 2
                robot2goalcounter = robot2goalcounter+1;
                end
            end
            
            if active==1 % select whether to run the code or not
                [field,robotstateret] = computeField(robot, robot, obst, goal, 'fireengine',i, tfxStaticObs, tfxDynamicObs, robotstate);
                robotstate = robotstateret;
                robotstateret(4,1);

                %{
                % model errors in measurement due to noise
                % additive white noise
                nweight = [0.004 0 0; 0 0.004 0; 0 0 0];
                delta = rand(3,1);
                %robot = robot+(delta*nweight)
                robot(i,:) = robot(i,:)+(nweight*delta)';
                %}
                
                % compute mag
                robot(i,:) = robot(i,:)-field';
                angle = atan2( (robot(i,2)-field(2))-robot(i,2), (robot(i,1)-field(1))-robot(i,1) )+(pi/2);
                
                % using hgtransform
                if(i==1)
                    hgFireEngine1.Matrix = makehgtform('translate',[robot(i,1),robot(i,2),0],'zrotate',(angle));
                end
                if(i==2)
                    hgFireEngine2.Matrix = makehgtform('translate',[robot(i,1),robot(i,2),0],'zrotate',(angle));
                end
                if(i==3)
                    hgFireEngine3.Matrix = makehgtform('translate',[robot(i,1),robot(i,2),0],'zrotate',(angle));
                end
                if(i==4)
                    hgFireEngine4.Matrix = makehgtform('translate',[robot(i,1),robot(i,2),0],'zrotate',(angle));
                end
            end
        end

        % loop related stuff after this only!
        %d2g = abs(norm(robotop(1,:))-norm(goal))
        d2g =  sqrt( (robot(1,1)-goal(1,1))^2 + (robot(1,2)-goal(1,2))^2 );
        axis([0 szGridX 0 szGridY]);
        pause(deltat);
        t = t+deltat;
        drawnow;
    end
end



