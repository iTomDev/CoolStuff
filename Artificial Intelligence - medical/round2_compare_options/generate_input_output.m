
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

