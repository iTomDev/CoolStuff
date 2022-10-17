
% Import data from excel spreadsheet and generate the target matrix 
t1 = xlsread('bacteria data.xls','sa0907');
t2 = xlsread('bacteria data.xls','sa1704');
t3 = xlsread('bacteria data.xls','ec1104');
t4 = xlsread('bacteria data.xls','ec1404');
t5 = xlsread('bacteria data.xls','sm1310');
% combine all the matrices
cwkinputs = ([t1 t2 t3 t4 t5]);
szcwkinputs = size(cwkinputs);
% generate a zero matrix: rows for classes, cols for samples
cwktargets = zeros(5,szcwkinputs(2));
% fill in ones to indicate class membership of each sample
% class 1
yposition = 0;
cwktargets(1,1:size(t1,2)) = ones(1,size(t1,2));
% class 2
yposition = size(t1,2);
cwktargets(2,yposition+1:yposition+size(t2,2)) = ones(1,size(t2,2));
% class 3
yposition = yposition+size(t2,2);
cwktargets(3,yposition+1:yposition+size(t3,2)) = ones(1,size(t3,2));
% class 4
yposition = yposition+size(t3,2);
cwktargets(4,yposition+1:yposition+size(t4,2)) = ones(1,size(t4,2));
% class 5
yposition = yposition+size(t4,2);
cwktargets(5,yposition+1:yposition+size(t5,2)) = ones(1,size(t5,2));



% generate mesh plot 
mesh(cwkinputs)

%% 
% lifting the data up
%Compensate for time decay in bacteria collection patterns
%The functions fits a straight line to each row of DATA,
%then pivots on the first curve and raises each row in turn
%until the straight line is horizontal
t1up = liftdata(t1);
t2up = liftdata(t2);
t3up = liftdata(t3);
t4up = liftdata(t4);
t5up = liftdata(t5);
cwkinputslift = ([t1up t2up t3up t4up t5up]);
mesh(cwkinputslift)

%
% plot before and after mesh plots
subplot(1,2,1)
mesh(cwkinputs)
title('Original')
subplot(1,2,2)
mesh(cwkinputslift)
title('Lifted')

%%
% statistical analysis 

linspace(1e3,5e7,37)

