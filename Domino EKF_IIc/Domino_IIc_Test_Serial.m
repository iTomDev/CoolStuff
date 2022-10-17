
TODEGREES = 57.2957795;
TORADIANS = 0.0174532925;

g = 9.80665;         % Earth Gravity Magnitude in m/s^2

% general
n=1; % loop counter
lc = 300; % number of loops
%dt = 0.1;%0.02;%0.5%0.09;
TrackPosQasE = zeros(3, lc);
StateXHistory = zeros(7, lc);
StateOMMHistory = zeros(3, lc);
StatePredHistory = zeros(3, lc);
MagHeadingSeries = zeros(1, lc);

%GPS_YAW = 90;
cog_gps = 0.05;%90*pi/180;

pitchx = 0;
rollx = 0;
     
Gsum = [0, 0, 0];

% Simulate system
% gyro rate
Gyro = [0.1, 0.1, -0.1];

% initial accelerometer position
Accel = [0, 0, 0]; %x, y, z NOT USED

time = zeros(lc);
w = [0, 0.0, 0.0]; % roll pitch yaw 
a = [0, 0, -1];
mag = [1, 0, 0];
init_heading = 0;
varianceA = 0.015;
varianceG = 0.01;
varianceH = 0.002;
%mag = [0, -0.66, 0];
GPSHeading = 0;
eulerhistory = zeros(3, lc);

StateX = [1, 0, 0, 0,  0, 0, 0];
P = zeros(7, 7);

%------- serial --------
    % Fetch data from IMU on serial connection
    % Reads into a data arrays formatted as
    % Channel 1 : AX
    % Channel 2 : AY
    % Channel 3 : AZ
    % Channel 4 : GYRO X
    % Channel 5 : GYRO Y
    % Channel 0 : GYRO Z (blank for now)
    s = serial('COM2');
    set(s, 'BaudRate', 9600);
    set(s, 'Databits', 8);
    set(s, 'InputBufferSize', 5000);
    %set(s, 'Terminator', 100);
    fopen(s);

    
    outputdata2old = [60; 0; 0; 0; 0; 0];

% ------------------ rotation display setup
% prepare graph
plotindex = 0:2;
%plot1 = plot(thisroll, thisroll);
%set(plot1,'XDataSource','thisroll');
%set(plot1,'YDataSource','thisroll');
%set(plot1,'Color','red','LineWidth',5);


%%
% Pitch plot

figure(1);
% create blank vectors
plotrollx = zeros(3);
plotrolly = zeros(3);
plotrollangle = 0;
% calculate the positions of x and y at each of the three points using the magic identity.
% y value is the sin(theta) and x value is the cos(theta)
plotrollx(1) = -cos(plotrollangle);
plotrollx(2) = 0;
plotrollx(3) = cos(plotrollangle);
plotrolly(1) = -sin(plotrollangle);
plotrolly(2) = 0;
plotrolly(3) = sin(plotrollangle);
% produce an array of x and y values from negative to the centre to the positive
plotrollx2 = [plotrollx(1), plotrollx(2), plotrollx(3)];
plotrolly2 = [plotrolly(1), plotrolly(2), plotrolly(3)];
% creates the plot and give it a handle
plotroll = plot(plotrollx2, plotrolly2);
% links the data in the graph
set(plotroll,'XDataSource','plotrollx2');
set(plotroll,'YDataSource','plotrolly2');
% sets the line colour
set(plotroll,'Color','red','LineWidth',3);
% sets the axes sizes
xlim([-2 2]);
ylim([-2 2]);

%%
% Plot for Pitch
figure(2);
plotpitchx = zeros(3);
plotpitchy = zeros(3);
plotpitchangle = 0;
% calculate the positions of x and y at each of the three points using the magic identity.
% y value is the sin(theta) and x value is the cos(theta)
plotpitchx(1) = -cos(plotpitchangle);
plotpitchx(2) = 0;
plotpitchx(3) = cos(plotpitchangle);
plotpitchy(1) = -sin(plotpitchangle);
plotpitchy(2) = 0;
plotpitchy(3) = sin(plotpitchangle);
% produce an array of x and y values from negative to the centre to the positive
plotpitchx2 = [plotpitchx(1), plotpitchx(2), plotpitchx(3)];
plotpitchy2 = [plotpitchy(1), plotpitchy(2), plotpitchy(3)];
% creates the plot and give it a handle
plotpitch = plot(plotpitchx2, plotpitchy2);
% links the data in the graph
set(plotpitch,'XDataSource','plotpitchx2');
set(plotpitch,'YDataSource','plotpitchy2');
% sets the line colour
set(plotpitch,'Color','blue','LineWidth',3);
% sets the axes sizes
xlim([-2 2]);
ylim([-2 2]);

%%
% Yaw plot

figure(3);
% create blank vectors
plotyawx = zeros(3);
plotyawy = zeros(3);
plotyawangle = 0;
% calculate the positions of x and y at each of the three points using the magic identity.
% y value is the sin(theta) and x value is the cos(theta)
plotyawx(1) = -cos(plotyawangle);
plotyawx(2) = 0;
plotyawx(3) = cos(plotyawangle);
plotyawy(1) = -sin(plotyawangle);
plotyawy(2) = 0;
plotyawy(3) = sin(plotyawangle);
% produce an array of x and y values from negative to the centre to the positive
plotyawx2 = [plotyawx(1), plotyawx(2), plotyawx(3)];
plotyawy2 = [plotyawy(1), plotyawy(2), plotyawy(3)];
% creates the plot and give it a handle
plotyaw = plot(plotyawx2, plotyawy2);
% links the data in the graph
set(plotyaw,'XDataSource','plotyawx2');
set(plotyaw,'YDataSource','plotyawy2');
% sets the line colour
set(plotyaw,'Color','green','LineWidth',3);
% sets the axes sizes
xlim([-2 2]);
ylim([-2 2]);
%------------------------------------

%P = ECM_Predict;

for n=1:lc
    
    % ------- serial --------
    fprintf(s, 1); % 3 could be any number
    % read 6 16b signed integers (accel, gyro)
    outputdata = fread(s, [12, 1], 'int16'); % was 8

    n;
    % display input as row
    outputdata';
    outputdata'; 
    % convert to degrees 
    outputdata2 = [0,0,0,0,0,0,0,0,0]';
    % 
    outputdata2(1) = outputdata(2) * 0.01101726;
    outputdata2(2) = outputdata(3) * 0.01101726;
    outputdata2(3) = outputdata(4) * 0.01101726;
    %
    outputdata2(4) = outputdata(6);% -127; % subtract 127 to set the centre at zero
    outputdata2(5) = outputdata(7);% -127;
    outputdata2(6) = outputdata(8);% -127; 
    % mag
    outputdata2(7) = outputdata(10);
    outputdata2(8) = outputdata(11);
    outputdata2(9) = outputdata(12);
    % display output as row
    outputdata2';
    outputdata2'; 
    
    % convert to radians 
%     outputdata2 = [0,0,0,0,0,0]';
%     outputdata2(1) = outputdata(1);
%     outputdata2(2) = outputdata(2);
%     outputdata2(3) = outputdata(3);
%     outputdata2(4) = outputdata(4) * 0.00019229;
%     outputdata2(5) = outputdata(5) * 0.00019229;
%     outputdata2(6) = outputdata(6) * 0.00019229;
%     % display output as row
%     outputdata2'; 
%     outputdata2'; 

    %{
    zeroingvals = [335; % Acc X. Centre: 340 to 243
                   329; % Acc Y. Centre: 336 to 337
                   334; % Acc Z. Centre: 352 to 355   354
                   277; % Gyro X. 280 to 283
                   275; % Gyro Y. 279 to 280
                   253]; %Gyro Z. 256 to 270


    outputdata2 = outputdata2 - zeroingvals;
    %}
    
    %outputdata2 = (outputdata2 + outputdata2old) / 2;
    %outputdata2old = outputdata2;
%     if -1 < outputdata2(4) < 1
%         outputdata2(4) = 0;
%     end
%     if -1 < outputdata2(5) < 1
%         outputdata2(5) = 0;
%     end
%     if -1 < outputdata2(6) < 1
%         outputdata2(6) = 0;
%     end
    
    
    outputdata2;  
    
%     gains = [0.01107794298635984, 0.01107794298635984, 0.01107794298635984, 0.01107794298635984, 0.01107794298635984, 0.01107794298635984];
%     outputdata2 = outputdata2 * gains;

    % gyro, and convert to radians
    G(1) = outputdata2(1)*TORADIANS; % roll
    G(2) = outputdata2(2)*TORADIANS; % pitch
    G(3) = outputdata2(3)*TORADIANS; % yaw
    
    % accel
%     G(4) = outputdata2(1);
%     G(5) = outputdata2(2);
%     G(6) = outputdata2(3);

%     G(4) = 0; %x 2
%     G(5) = 1; %y -1
%     G(6) = -1; %z 3
    
    % fixed for plug in dev board which isnt aligned
%     G(4) = outputdata2(6); %x 2
%     G(5) = -outputdata2(5); %y -1
%     G(6) = outputdata2(4); %z 3

    G(4) = -outputdata2(4); %x 2
    G(5) = outputdata2(5); %y -1
    G(6) = outputdata2(6); %z 3

    
    % normalise accel
    accelmag = sqrt(G(4)^2 + G(5)^2 + G(6)^2);
%     accelmag = G(4) + G(5) + G(6);
    G(4) = G(4) / accelmag;
    G(5) = G(5) / accelmag;
    G(6) = G(6) / accelmag;
%     G(4) = G(4) / 127;
%     G(5) = G(5) / 127;
%     G(6) = G(6) / 127;
    
    G(4) = G(4) * g;
    G(5) = G(5) * g;
    G(6) = G(6) * g;
    
    % subtract some bias
    G(1) = G(1) - 0;
    G(2) = G(2) - 0;
    G(3) = G(3) - 0;
    
    % Z and Y switched as that is the order the mag sends then, change
    % this using the firmware
    % flip X sign as mag is aligned with the accel IIRC!
    
%     G2(7) = -outputdata2(7); % X
%     G2(8) = outputdata2(9); % Y
%     G2(9) = outputdata2(8); % Z
    
    G2(7) = -outputdata2(7); % X
    G2(8) = outputdata2(9); % Y
    G2(9) = outputdata2(8); % Z
    
    magmag = sqrt(G2(7)^2 + G2(8)^2 + G2(9)^2);
    G(7) = G2(7) / magmag;
    G(8) = G2(8) / magmag;
    G(9) = G2(9) / magmag;
    
    IMUData = [G(4), G(5), G(6), G(7), G(8), G(9)];
    IMUData;
    
    %----- serial ends
%     fclose(s);
%     delete(s);

% Gyro
%     G(1) = 0.01;
%     G(2) = 0.01;
%     G(3) = 0;
% Accel    
%     G(4) = 1;
%     G(5) = 0;
%     G(6) = 0;
    
    % sum the input angles
%     Gsum(1) = Gsum(1) + (G(1)*dt*57);
%     Gsum(2) = Gsum(2) + (G(2)*dt*57);
%     Gsum(3) = Gsum(3) + (G(3)*dt*57);
%     Gsum;
    
%     if n>100
%         cog_gps = 0.5*pi;
%     end
%     
%     if n>200
%         cog_gps = 1*pi;
%     end
%     
%     if n>300
%         cog_gps = 1.5*pi;
%     end
%     
%     if n>400
%         cog_gps = 2*pi;
%     end
    

    w = [G(1), G(2), G(3)];
    a = [G(4), G(5), G(6)];
    mag = [G(7), G(8), G(9)];
    
%     w = [-0.1, 0, 0.0];
%     a = [0, 0, -1];
%     
%     init_heading = 0.01;
%     varianceA = 0.0001;
%     varianceG = 0.0001;
%     varianceH = 0.0001;
%     %mag = [0, -0.66, 0];
     GPSHeading = GPSHeading + 0.001;
    
    wvar = 0.0001;
    avar = 0.0001;
    dt = 0.04;
    ghvar = 0.0001;
    gps_velocity = 1;
    mvar = 0.0001;
    
    % call filter
    %[StateX, StateOMM, StatePred] = EKF_GyroAcc_V3(StateX, dt, Gyro, accel GPS_YAW, ECM_Predict, StatePred);
    %[ StateX, P, SXF ] = EKF_Paul_S_V4_Serial(StateX', dt, [G(1), G(2), G(3)]', [G(4), G(5), G(6)]', P, cog_gps, SXF);
    [StateX, P] = Domino_IIc(dt, StateX, P, w, wvar, a, avar, GPSHeading, ghvar, gps_velocity, mag, mvar );
    
    % maintain history of state vals
%    StateXHistory(:, n) = StateX';
    
    StateX;
    
    % convert quaternion position back to euler 
    PosQ = [StateX(1), StateX(2), StateX(3), StateX(4)]; 
    PosQasE(1) = atan2(2*(PosQ(3)*PosQ(4)+PosQ(1)*PosQ(2)), (1-2*((PosQ(2)^2)+(PosQ(3)^2))));
    PosQasE(2) = -asin(2*(PosQ(2)*PosQ(4)-PosQ(1)*PosQ(3)));
    PosQasE(3) = atan2(2*(PosQ(2)*PosQ(3)+PosQ(1)*PosQ(4)), (1-2*((PosQ(3)^2)+(PosQ(4)^2))));
    % collect the eulers for graphing over time
    DisplayStateXasE = [PosQasE(1)*180/pi, PosQasE(2)*180/pi, PosQasE(3)*180/pi];
    %TrackPosQasE(1, n) = PosQasE(1) ;
    TrackPosQasE(1, n) = PosQasE(1) *180/pi;
    TrackPosQasE(2, n) = PosQasE(2) *180/pi;
    TrackPosQasE(3, n) = PosQasE(3) *180/pi;
    
    pitchx = PosQasE(1);
    rollx = PosQasE(2);
    PosQasE(3);
    
    StateOMM = [0,0,0];
    StateOMM(1) = StateOMM(1)*180/pi;
    StateOMM(2) = StateOMM(2)*180/pi;
    StateOMM(3) = StateOMM(3)*180/pi;
    StateOMMHistory(:, n) = StateOMM;
    
    %StatePredGraph = StatePred;
    
    % make eulers for monitoring output of estimation
    %PosQ = [StatePred(1), StatePred(2), StatePred(3), StatePred(4)];
    PosQ = [StateX(1), StateX(2), StateX(3), StateX(4)] ;
    StatePredGraph(1) = atan2(2*(PosQ(3)*PosQ(4)+PosQ(1)*PosQ(2)), (1-2*((PosQ(2)^2)+(PosQ(3)^2))));
    StatePredGraph(2) = -asin(2*(PosQ(2)*PosQ(4)-PosQ(1)*PosQ(3)));
    StatePredGraph(3) = atan2(2*(PosQ(2)*PosQ(3)+PosQ(1)*PosQ(4)), (1-2*((PosQ(3)^2)+(PosQ(4)^2))));
    PosQ = 0;
    
%     StatePredGraph(1) = StatePredGraph(1)*180/pi;
%     StatePredGraph(2) = StatePredGraph(2)*180/pi;
%     StatePredGraph(3) = StatePredGraph(3)*180/pi;
%     StatePredHistory(:, n) = StatePredGraph;
    StatePredHistory(1, n) = StatePredGraph(1)*180/pi;
    StatePredHistory(2, n) = StatePredGraph(2)*180/pi;
    StatePredHistory(3, n) = StatePredGraph(3)*180/pi;
    % prepare for next state 
    % compute new accelerometer angle
    
    %--------------- rotation display ---------------------
    plotrollangle = PosQasE(1);
    plotpitchangle = PosQasE(2);
    plotyawangle = PosQasE(3);

    % roll plot update
    % calculate the positions of x and y at each of the three points using the magic identity.
    % y value is the sin(theta) and x value is the cos(theta)
    plotrollx(1) = -cos(plotrollangle);
    plotrollx(2) = 0;
    plotrollx(3) = cos(plotrollangle);
    plotrolly(1) = -sin(plotrollangle);
    plotrolly(2) = 0;
    plotrolly(3) = sin(plotrollangle);
    % produce an array of x and y values from negative to the centre to the positive
    plotrollx2 = [plotrollx(1), plotrollx(2), plotrollx(3)];
    plotrolly2 = [plotrolly(1), plotrolly(2), plotrolly(3)];

    % pitch plot update
    plotpitchx(1) = -cos(plotpitchangle);
    plotpitchx(2) = 0;
    plotpitchx(3) = cos(plotpitchangle);
    plotpitchy(1) = -sin(plotpitchangle);
    plotpitchy(2) = 0;
    plotpitchy(3) = sin(plotpitchangle);
    plotpitchx2 = [plotpitchx(1), plotpitchx(2), plotpitchx(3)];
    plotpitchy2 = [plotpitchy(1), plotpitchy(2), plotpitchy(3)];

    % yaw plot update
    plotyawx(1) = -cos(plotyawangle);
    plotyawx(2) = 0;
    plotyawx(3) = cos(plotyawangle);
    plotyawy(1) = -sin(plotyawangle);
    plotyawy(2) = 0;
    plotyawy(3) = sin(plotyawangle);
    plotyawx2 = [plotyawx(1), plotyawx(2), plotyawx(3)];
    plotyawy2 = [plotyawy(1), plotyawy(2), plotyawy(3)];

    %plot1 = plot(plotx, ploty);

    pause(0.01); % pause so data can be viewed between updates
    refreshdata(plotroll); % refresh graph
    refreshdata(plotpitch); % refresh graph
    refreshdata(plotyaw); % refresh graph
    %----------------------------------------------
    
    %pitchx = plotyawangle;
    %rollx = plotrollangle;
end

fclose(s);
delete(s);

% make a comma delimited file for animation in Flightgear
% row 1: Time Longitude Latitude Altitude Roll Pitch Heading
% row 2 ect are data
% 40.275	-122.480847	37.665543	7224	0	0.424	117.9
fgdata = fopen('C:\Program Files\MATLAB\R2010b\toolbox\aero\astdemos\fsimlog.csv', 'w'); % w - overwrite existing
fprintf(fgdata, 'Time, Longitude, Latitude, Altitude, Roll, Pitch, Heading\n');
latitude = 37.665543;
for n=1:lc
    latitude = latitude + 0.001;
    fprintf(fgdata, '%f,', n); % time, /50 because 0.02 = 1/50
    fprintf(fgdata, '%f,', -122.480847); % longitude
    fprintf(fgdata, '%f,', latitude); %latitude
    fprintf(fgdata, '%f,', 7224); % altitude
    fprintf(fgdata, '%f,', (TrackPosQasE(1, n))); % roll
    fprintf(fgdata, '%f,', (TrackPosQasE(2, n))); % pitch
    fprintf(fgdata, '%f\n', (TrackPosQasE(3, n))); %heading
end
fclose(fgdata);

n=1:lc;
% figure;
% subplot(4,1,1);plot(n, TrackPosQasE(1, :), 'r', n, TrackPosQasE(2, :), 'c', n, TrackPosQasE(3, :), 'b'); 
% title('StateX Output in Eulers')
% subplot(4,1,2);plot(n, StateOMMHistory(1, :), 'r', n, StateOMMHistory(2, :), 'c', n, StateOMMHistory(3, :), 'b');
% title('"update" component only. In Eulers, includes others after 1st loop')
% subplot(4,1,3);plot(n, StatePredHistory(1, :), 'r', n, StatePredHistory(2, :), 'c', n, StatePredHistory(3, :), 'b');
% title('"prediction" component only. In Eulers')

% red = roll, aqua = pitch, blue = yaw
plot(n, TrackPosQasE(1, :), 'r', n, TrackPosQasE(2, :), 'c', n, TrackPosQasE(3, :), 'b'); 
%plot(n, TrackPosQasE(1, :), 'r', n, TrackPosQasE(2, :), 'c');  % plot witout yaw!