function [] = nextSeqLED(epic, inputArg1)
%NEXTSEQLED goes through a sequence of led each time its called
% sequence is
%    0
% 1 and 7
% 6 and 2
% 5 and 3
% 4

if(inputArg1==0)
    % led 4 off
    epic=set(epic,'ledOFF',[4]);
    epic=update(epic);
    % led 0 on
    epic=set(epic,'ledON',[0]);
    epic=update(epic);
end

if(inputArg1==1)
    % led off
    epic=set(epic,'ledOFF',[0]);
    epic=update(epic);
    % led on
    epic=set(epic,'ledON',[1]);
    epic=update(epic);
    epic=set(epic,'ledON',[7]);
    epic=update(epic);
end

if(inputArg1==2)
    % led off
    epic=set(epic,'ledOFF',[1]);
    epic=update(epic);
    epic=set(epic,'ledOFF',[7]);
    epic=update(epic);
    % led on
    epic=set(epic,'ledON',[6]);
    epic=update(epic);
    epic=set(epic,'ledON',[2]);
    epic=update(epic);
end

if(inputArg1==3)
    % led off
    epic=set(epic,'ledOFF',[2]);
    epic=update(epic);
    epic=set(epic,'ledOFF',[6]);
    epic=update(epic);
    % led on
    epic=set(epic,'ledON',[3]);
    epic=update(epic);
    epic=set(epic,'ledON',[5]);
    epic=update(epic);
end

if(inputArg1==4)
    % led off
    epic=set(epic,'ledOFF',[3]);
    epic=update(epic);
    epic=set(epic,'ledOFF',[5]);
    epic=update(epic);
    % led 0 on
    epic=set(epic,'ledON',[4]);
    epic=update(epic);
end

end

