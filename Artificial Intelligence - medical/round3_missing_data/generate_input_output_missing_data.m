
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

%%
% remove data at random from each set
% 111 random missing measurements

% generate as many randoms as there are experiments
r1 = rand(1,287);
r2 = rand(1,287);
r3 = rand(1,287);
% normalise to the number of measurements
% this tell us which measurement to delete, randomly
r1 = round(36*r1);
r1 = r1+1;
r2 = round(36*r2);
r2 = r2+1;
r3 = round(36*r3);
r3 = r3+1;
for i=1:287
    cwkinputs(r1(1,i),i) = 0;
    cwkinputs(r2(1,i),i) = 0;
    cwkinputs(r3(1,i),i) = 0;
end
% verify that it worked
cwkinputs(:,1)'



%%

% a few consecutive losses too, 5x5
% 1 to 57
% cwkinputs(11:15,11) = zeros(5,1);
% cwkinputs(21:25,22) = zeros(5,1);
% cwkinputs(4:8,44) = zeros(5,1);

% 5 consecutive missing samples in 3 experiments from each series. 6*3*5=90
% so 90 missing measurements
% 1:57
cwkinputs(16:20,8) = 0;
cwkinputs(5:9,33) = 0;
cwkinputs(25:29,48) = 0;
% 58:117
cwkinputs(8:12,66) = 0;
cwkinputs(8:12,88) = 0;
cwkinputs(8:12,108) = 0;
% 118:177
cwkinputs(16:20,120) = 0;
cwkinputs(5:9,140) = 0;
cwkinputs(25:29,160) = 0;
% 178:227
cwkinputs(15:19,22) = 0;
cwkinputs(24:28,22) = 0;
cwkinputs(6:10,22) = 0;
% 227:287
cwkinputs(26:30,238) = 0;
cwkinputs(6:10,258) = 0;
cwkinputs(16:20,278) = 0;
%}

%%
% attempt to fix data
cwkinputsfixed = zeros(size(cwkinputs));
for cwn=1:size(cwkinputs,2)
    cwkinputsfixed(:,cwn) = fixdata(cwkinputs(:,cwn));
end

