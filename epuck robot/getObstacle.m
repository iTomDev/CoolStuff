function [obslist, obsi2] = getObstacle(obslist, proxivals, position, obsi)
%Takes the proximity sensor values and returns a very rough location of the
%obstacle 

% 2000: next to
% 600: 10mm
% 250: 20mm
% 100: 30mm
% 25: 40mm
% 5: 50mm

% obstacle in front
if(proxivals(8)>24 && proxivals(1)>24)
    % add front obstacle
    meas = (proxivals(8)+proxivals(1))/2;
    if meas>5
        dist = 50;
    elseif meas>25
        dist = 40;
    elseif meas>100
        dist = 30;
    elseif meas>250
        dist = 20;
    elseif meas>500
        dist = 10;
    elseif meas>1100
        dist = 0;
    end
    obslist(obsi,:) = position(1:2)+[dist 0];
    obsi = obsi+1;
end

% obstacle to left
if(proxivals(6)>24)
    % add front obstacle
    meas = proxivals(6);
    if meas>5
        dist = 50;
    elseif meas>25
        dist = 40;
    elseif meas>100
        dist = 30;
    elseif meas>250
        dist = 20;
    elseif meas>500
        dist = 10;
    elseif meas>1100
        dist = 0;
    end
    obslist(obsi,:) = position(1:2)+[0 -dist];
    obsi = obsi+1;
end

% obstacle to right
if(proxivals(3)>24)
    % add front obstacle
    meas = proxivals(3);
    if meas>5
        dist = 50;
    elseif meas>25
        dist = 40;
    elseif meas>100
        dist = 30;
    elseif meas>250
        dist = 20;
    elseif meas>500
        dist = 10;
    elseif meas>1100
        dist = 0;
    end
    obslist(obsi,:) = position(1:2)+[0 dist];
    obsi = obsi+1;
end

% obstacle to alternative for right hand side 
%(one sensor isnt on rhs doesnt work right)
if(proxivals(2)>70)
    % add front obstacle
    meas = proxivals(2);
    if meas>70
        dist = 40;     
    elseif meas>80
        dist = 30;
    elseif meas>150
        dist = 20;
    elseif meas>200
        dist = 10;
    end
    obslist(obsi,:) = position(1:2)+[0 dist];
    obsi = obsi+1;
end



obsi2 = obsi;
