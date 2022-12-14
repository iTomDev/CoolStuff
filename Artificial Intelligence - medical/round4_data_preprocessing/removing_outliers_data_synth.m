% Solve a Pattern Recognition Problem with a Neural Network
% Script generated by Neural Pattern Recognition app
% Created 02-Mar-2018 20:35:29
%
% This script assumes these variables are defined:
%
%   cwkinputs - input data.
%   cwktargets - target data.

% remove the outliers
% process all blocks
t1outs = removeoutliers( t1 );
t2outs = removeoutliers( t2 );
t3outs = removeoutliers( t3 );
t4outs = removeoutliers( t4 );
t5outs = removeoutliers( t5 );

cwktargetsouts = [t1outs t2outs t3outs t4outs t5outs];

xin = cwktargetsouts;

t = cwktargets;

% Choose a Training Function
% For a list of all training functions type: help nntrain
% 'trainlm' is usually fastest.
% 'trainbr' takes longer but may be better for challenging problems.
% 'trainscg' uses less memory. Suitable in low memory situations.
trainFcn = 'trainbr';  % Scaled conjugate gradient backpropagation.
%trainFcn = 'trainscg';

% Create a Pattern Recognition Network
hiddenLayerSize = 6;
net = patternnet(hiddenLayerSize);

% change activation function - me
% for i=1:net.numLayers
%     if strcmp(net.layers{i}.transferFcn,'tansig')
%         net.layers{i}.transferFcn = 'logsig';
%         %net.layers{i}.transferFcn = 'purelin';
%         %net.layers{i}.transferFcn = 'elliotsig';
%         %net.layers{i}.transferFcn = 'tansig';
%     end
% end

% added by me 
net.trainParam.goal = 0;
%net.trainParam.mu = 0.001;

% Choose Input and Output Pre/Post-Processing Functions
% For a list of all processing functions type: help nnprocess
net.input.processFcns = {'removeconstantrows','mapminmax'};
net.output.processFcns = {'removeconstantrows','mapminmax'};

% Setup Division of Data for Training, Validation, Testing
% For a list of all data division functions type: help nndivide
net.divideFcn = 'dividerand';  % Divide data randomly
net.divideMode = 'sample';  % Divide up every sample
net.divideParam.trainRatio = 70/100;
net.divideParam.valRatio = 15/100;
net.divideParam.testRatio = 15/100;

% Choose a Performance Function
% For a list of all performance functions type: help nnperformance
net.performFcn = 'crossentropy';  % Cross-Entropy

% Choose Plot Functions
% For a list of all plot functions type: help nnplot
net.plotFcns = {'plotperform','plottrainstate','ploterrhist', ...
    'plotconfusion', 'plotroc'};

% timing
start = cputime;

% best and worse nets so far
minPercentErrorVal = inf;
minPercentErrorNet = 0;
maxPerformanceVal = 0;
maxPerformanceNet = 0;
maxPerformancetr = 0;
minPerformanceVal = inf;
minPerformanceNet = 0;
minPerformancetr = 0;

runs = 50;
netArr = zeros(3,runs);
for i=1:runs

    net = init(net); 
    
    % scale inputs so they are in the range -1 to 1
    %[pn, minp, maxp, tn, mint, maxt] = premnmx(p,t);
    %x = mapminmax(xin);
    x = xin;
    
    % Train the Network
    [net,tr] = train(net,x,t);

    % Test the Network
    y = net(x);
    e = gsubtract(t,y);
    performance = perform(net,t,y);
    tind = vec2ind(t);
    yind = vec2ind(y);
    percentErrors = sum(tind ~= yind)/numel(tind);
    
    % compile best and worse networks
    % lowest percentage error
    if (percentErrors < minPercentErrorVal)
        minPercentErrorVal = percentErrors;
        minPercentErrorNet = net;
    end
    % highest performance
    if (performance > maxPerformanceVal)
        maxPerformanceVal = performance;
        maxPerformanceNet = net;
        maxPerformancetr = tr;
    end
    % lowest performance
    if (performance < minPerformanceVal)
        minPerformanceVal = performance;
        minPerformanceNet = net;
        minPerformancetr = tr;
    end
    
    % compile into an array
    netArr(1,i) = performance;
    netArr(2,i) = percentErrors;
    netArr(3,i) = confusion(t,y); % confusion value. percentage misclassified
end
% display time taken
timetaken = cputime-start;
timetaken
disp('secs')

% plot the rests of the tests
figure(1)
i = 1:1:runs;
subplot(2,2,1)
% plot highest perofmrnace
scatter(i(:), netArr(1,i))
[mx mi] = max(netArr(1,i));
text(mi+1,mx,'Highest Perf','Color','green')
[mn mi] = min(netArr(1,i));
text(mi+1,mn,'Lowest Perf','Color','red')
fit = polyfit(i,netArr(1,:),1);
line([0 50],[fit(1) fit(2)],'Color','m','LineStyle','--')
title('performance')
xlabel('test run')
subplot(2,2,2)
scatter(i(:), netArr(2,i))
xlabel('test run')
title('percent error');
subplot(2,2,3)
scatter(i(:), netArr(3,i))
xlabel('test run')
title('confusion factor');

% plot worst and best performnce networks
% confusion plots
figure(1)
plotconfusion(t,maxPerformanceNet(x))
figure(2)
plotconfusion(t,minPerformanceNet(x)) % this is "best"
% plot performance graphs for the performance network
figure(3)
plotperform(maxPerformancetr)
figure(4)
plotperform(minPerformancetr)

% Recalculate Training, Validation and Test Performance
    trainTargets = t .* tr.trainMask{1};
    valTargets = t .* tr.valMask{1};
    testTargets = t .* tr.testMask{1};
    trainPerformance = perform(net,trainTargets,y)
    valPerformance = perform(net,valTargets,y)
    testPerformance = perform(net,testTargets,y)

% View the Network
view(net)

% plot results for the net
view(net)

% Plots
% Uncomment these lines to enable various plots.
%figure, plotperform(tr)
%figure, plottrainstate(tr)
%figure, ploterrhist(e)
figure, plotconfusion(t,y)
%figure, plotroc(t,y)

% Deployment
% Change the (false) values to (true) to enable the following code blocks.
% See the help for each generation function for more information.
if (false)
    % Generate MATLAB function for neural network for application
    % deployment in MATLAB scripts or with MATLAB Compiler and Builder
    % tools, or simply to examine the calculations your trained neural
    % network performs.
    genFunction(net,'myNeuralNetworkFunction');
    y = myNeuralNetworkFunction(x);
end
if (false)
    % Generate a matrix-only MATLAB function for neural network code
    % generation with MATLAB Coder tools.
    genFunction(net,'myNeuralNetworkFunction','MatrixOnly','yes');
    y = myNeuralNetworkFunction(x);
end
if (false)
    % Generate a Simulink diagram for simulation or deployment with.
    % Simulink Coder tools.
    gensim(net);
end

% check what the frequence spacings are
range = 5E7-1E3;
for o=1:37
    range = range/2;
end
range


