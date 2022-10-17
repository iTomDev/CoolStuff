
epic = ePicKernel;
epic = connect(epic,'COM46');
epic = update(epic);

%%
% performs obstacle avoidance, and some fairly unreliable obstacle mapping
% and route mapping

figure(1)
title('Environment')
hold on
axis([-200 200 -200 200])

% sense and avoid - works!
ledsequenceno = 0;

% set up position
pos = zeros(1000,3);
encoderprev = zeros(1,2);
encodernow = zeros(1,2);
%pos = zeros(10,2);
posi = 2;
epic=updateDef(epic,'pos',1);
[val,up] = get(epic,'pos');
epic = update(epic);
enc0 = val;

% obstacle handling
obslist = zeros(100,2);
obsi = 1;

%fwdspeed = 200;
fwdspeed = 300;
trnspeed = 400;
revspeed = 300;

for lp=1:1:40
    % drive a bit
    epic = set(epic,'speed',[fwdspeed,fwdspeed]);
    epic = update(epic);
    %pause(0.1);
    % update position
    encoderprev = encodernow;
    [val,up] = get(epic,'pos');
    encodernow = val - enc0;
    posi = posi + 1;
    pos(posi,:) = getPosition(pos(posi-1,:), encoderprev, encodernow);
   
    proxiAvg = zeros(1,8);
    for i=1:1:1
        epic = update(epic);
        proxiVal = get(epic,'proxi');
        proxiAvg = (proxiVal);% + proxiAvg)/2;
    end
    proxiAvg;
    [obslist,obsi] = getObstacle(obslist,proxiAvg,pos(posi,:),obsi);
    
    % proximity sensor - look for obsts
    %epic = set(epic,'speed',[0,0]);
    %epic = update(epic);
    
    % there is an obstacle in front - go around!
    if(proxiAvg(8)>50 || proxiAvg(1)>50)
        % reverse a bit
        %epic = set(epic,'speed',[-revspeed,-revspeed]);
        %epic = update(epic);
        %pause(1);
        %epic = set(epic,'speed',[0,0]);
        %epic = update(epic);
        % update position
        encoderprev = encodernow;
        [val,up] = get(epic,'pos');
        encodernow = val - enc0;
        pos(posi,:) = getPosition(pos(posi-1,:), encoderprev, encodernow);
        posi = posi + 1;
        %
            
        % left has no obstacle
        if(proxiAvg(2)<50 && proxiAvg(3)<50 && proxiAvg(3)<proxiAvg(6))
            % turn right
            epic = set(epic,'speed',[trnspeed,0.1*trnspeed]);
            epic = update(epic);
            pause(2.6);
            epic = set(epic,'speed',[0,0]);
            epic = update(epic);
            % update position
            encoderprev = encodernow;
            [val,up] = get(epic,'pos');
            encodernow = val - enc0;
            pos(posi,:) = getPosition(pos(posi-1,:), encoderprev, encodernow);
            posi = posi + 1;
        %end
        % right has no obstacle
        elseif(proxiAvg(7)<50 && proxiAvg(6)<50)
            % turn left
            epic = set(epic,'speed',[0.1*trnspeed,trnspeed]);
            epic = update(epic);
            pause(2.6);
            epic = set(epic,'speed',[0,0]);
            epic = update(epic);
            % update position
            encoderprev = encodernow;
            [val,up] = get(epic,'pos');
            encodernow = val - enc0;
            pos(posi,:) = getPosition(pos(posi-1,:), encoderprev, encodernow);
            posi = posi + 1;
            %
        end
    end
    
    %{
    % refresh distances
    %epic = set(epic,'speed',[0,0]);
    %epic = update(epic);
    proxiAvg = zeros(1,8);
    for i=1:1:1
        epic = update(epic);
        proxiVal = get(epic,'proxi');
        proxiAvg = (proxiVal + proxiAvg)/2;
    end
    proxiAvg;
    [obslist,obsi] = getObstacle(obslist,proxiAvg,pos(posi,:),obsi);
    %}
    
    
    % left has an obstacle
    if(proxiAvg(7)>250)
        % reverse a bit
        epic = set(epic,'speed',[-revspeed,-revspeed]);
        epic = update(epic);
        pause(1);
        %epic = set(epic,'speed',[0,0]);
        %epic = update(epic);
        % update position
        encoderprev = encodernow;
        [val,up] = get(epic,'pos');
        encodernow = val - enc0;
        pos(posi,:) = getPosition(pos(posi-1,:), encoderprev, encodernow);
        posi = posi + 1;
        %
        % turn right
        epic = set(epic,'speed',[trnspeed,0]);
        epic = update(epic);
        pause(1);
        %epic = set(epic,'speed',[0,0]);
        %epic = update(epic);
        % update position
        encoderprev = encodernow;
        [val,up] = get(epic,'pos');
        encodernow = val - enc0;
        pos(posi,:) = getPosition(pos(posi-1,:), encoderprev, encodernow);
        posi = posi + 1;
        %
    end
    
    % right has an obstacle
    if(proxiAvg(2)>250)
        % reverse a bit
        epic = set(epic,'speed',[-revspeed,-revspeed]);
        epic = update(epic);
        pause(1);
        %epic = set(epic,'speed',[0,0]);
        %epic = update(epic);
        % update position
        encoderprev = encodernow;
        [val,up] = get(epic,'pos');
        encodernow = val - enc0;
        pos(posi,:) = getPosition(pos(posi-1,:), encoderprev, encodernow);
        posi = posi + 1;
        %
        % turn right
        epic = set(epic,'speed',[0,trnspeed]);
        epic = update(epic);
        pause(1);
        %epic = set(epic,'speed',[0,0]);
        %epic = update(epic);
        % update position
        encoderprev = encodernow;
        [val,up] = get(epic,'pos');
        encodernow = val - enc0;
        pos(posi,:) = getPosition(pos(posi-1,:), encoderprev, encodernow);
        posi = posi + 1;
        %
    end
    
    % LED sequencing
    nextSeqLED(epic,ledsequenceno);
    ledsequenceno = ledsequenceno + 1;
    if(ledsequenceno>4)
        ledsequenceno = 0;
    end
    
    
    
    % update gui
    if (lp>1)
        drawLine(pos(lp-1,2),pos(lp-1,1),pos(lp,2),pos(lp,1),'blue');
        %refresh;
        drawnow;
        % draw obstacles
        if(obsi>1)
            %drawCircle(obslist(obsi-1,2),obslist(obsi-1,1),12);
        end
    end
end


epic = set(epic,'speed',[0,0]);
epic = update(epic);
        
allLEDOff(epic)

%%

%display map at the end
for i=1:1:posi
    drawLine(pos(i,1),pos(i,2),pos(i+1,1),pos(i+1,2),'blue')
end
for j=2:1:obsi
    drawCircle(obslist(obsi-1,2),obslist(obsi-1,1),12);
end
axis([-200 200 -200 200])

%%
epic = disconnect(epic);
