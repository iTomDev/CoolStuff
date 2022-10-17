
% Thomas Pile, 21048743

% Note: Black background, white line
% I don't have any marker pens :)


epic = ePicKernel;
epic = connect(epic,'COM11');
epic = update(epic);

%%

% program vars
ledsequenceno = 0;

% activate line sensors
epic = updateDef(epic, 'floor', 1);
speed = 200;
% turn speeds
speedh = 500;%700;%300;
speedl = 200;%100;

b = 50 ;
journey = zeros(3,100);
xytheta = zeros(3,1);

encoderprev = [0 0];
encodernow = [0 0];
epic=updateDef(epic,'pos',1);
[val,up] = get(epic,'pos');
epic = update(epic);
enc0 = val;
theta = 0;

for lp=1:1:70
    % check line locations
    floor = get(epic,'floor');
    epic = update(epic);
    
    %------------- odom
    encoderprev = encodernow;
    %epic = update(epic);
    [val,up] = get(epic,'pos');
    epic = update(epic);
    encodernow = val - enc0;
    
    % distance travelled by each wheel in deltaT
    dsl = (encodernow(1)-encoderprev(1));
    dsr = (encodernow(2)-encoderprev(2));
    % calcs
    xythetad = [((dsr+dsl)/2) * cos(theta+((dsl-dsr)/(2*b)));
                ((dsr+dsl)/2) * sin(theta+((dsl-dsr)/(2*b)));
                ((dsr-dsl)/b)];
    xythetad;
    %
    theta = xytheta(3);
    xytheta = xytheta + xythetad;
    xytheta = xytheta*0.01;
    journey(:,lp) = xytheta(:);
    
    
    if (floor(2)<500)
        % drive straight
        epic = set(epic,'speed',[speed,speed]);
        epic = update(epic);
        %pause(0.5);
        %epic = set(epic,'speed',[0,0]);
        %epic = update(epic);
    end
    
    if (floor(3)<500)
        % drive straight and a bit left
        epic = set(epic,'speed',[speedh,speedl]);
        epic = update(epic);
        %pause(0.5);
        %epic = set(epic,'speed',[0,0]);
        %epic = update(epic);
    else if (floor(1)<500)
            % drive straight and a bit right
            epic = set(epic,'speed',[speedl,speedh]);
            epic = update(epic);
            %pause(0.5);
            %epic = set(epic,'speed',[0,0]);
            %epic = update(epic);
        end
    end
    
    %{
    % stop if all white
    if (floor(1)>500 && floor(2)>500 && floor(3)>500)
        pause(2)
        if (floor(1)>500 && floor(2)>500 && floor(3)>500)
            allLEDOff(epic);
            epic = set(epic,'speed',[0,0]);
            epic = update(epic);
            break;
        end
    end
    %}
    pause(0.05);
    
    % LED sequencing
    
    nextSeqLED(epic,ledsequenceno);
    ledsequenceno = ledsequenceno + 1;
    if(ledsequenceno>4)
        ledsequenceno = 0;
    end
    
    
    
end

allLEDOff(epic);
epic = set(epic,'speed',[0,0]);
epic = update(epic);

%%

plot(journey(2,:),journey(1,:))
xlabel('Y Position')
ylabel('X Position');

%%
epic = disconnect(epic);