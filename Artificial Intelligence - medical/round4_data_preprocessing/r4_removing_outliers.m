% This code is for R&D!!!!

plot(t1(:,1))
std(t1(:,1))

mean  = mean(t1(:,1))
mean(t1(:,1),1) % equiv
mean(t1(:,:),1) % equiv. gives the mean across all tests for each measurement point

s = std(t1(1,:),1) % equiv
std(t1,2) % equiv. 

plot([1:37]',t1(:,1))
line([1 10],[mean-s mean+s])

%%
% mean and standard deviation development

% mean measurement 1 across the 50 tests
mn  = mean(t1(1,:));
s = std(t1(1,:),1);
%size(t1(1,:))
plot([1:57]',t1(1,:)');
% histogram
histogram(t1(1,:),50);
% histogram with normal distribution, 10 bins
hfit = histfit(t1(1,:),10);

%%
% apply to entire measurement sets
% change to t1, t2, t3, t4, t5 

% standard deviation for all 5 measurements
dataset = t1(:,:);
s_all = std(dataset',1); % so we can go the other way through the array!
mn_all = mean(dataset,2); % 2 defines direction in the array

% plot standard deviation for each measurement of the 37, and then the
% upper and lower 2 sigma bounds with lines
upperbnd = mn_all+(2*s_all)';
lowerbnd = mn_all-(2*s_all)';
plot([1:37]',mn_all,[1:37]',upperbnd,[1:37]',lowerbnd)
title('Mean and 2 Sigma for Dataset 1')

%% 
% removing the outliers for the above dataset


%size(dataset)
% move through measurement values e.g 1:37
newset = zeros(size(dataset));
for rws = 1:size(dataset,1)
    % move through each value at that measurement e.g 1:57
    for cls = 1:size(dataset,2)
        % check whether the value is out of the sigma bounds
        % if it is, replace with the mean
        if abs(dataset(rws,cls)) > upperbnd(rws)
            newset(rws,cls) = mn_all(rws);
        else
            newset(rws,cls) = dataset(rws,cls);
        end
    end
end

% compare fixed to new dataset
plot([1:57]',dataset(1,:)','r-',[1:57]',newset(1,:)','g-')
legend('Original Data','Outliers Removed')
title('Removing Outliers')

