function [ newset ] = remoteoutliers( dataset )
%REMOTEOUTLIERS Summary of this function goes here
%   Detailed explanation goes herenewset = zeros(size(dataset));

s_all = std(dataset',1); % so we can go the other way through the array!
mn_all = mean(dataset,2); % 2 defines direction in the array

% plot standard deviation for each measurement of the 37, and then the
% upper and lower 2 sigma bounds with lines
upperbnd = mn_all+(2*s_all)';
lowerbnd = mn_all-(2*s_all)';

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
%plot([1:57]',dataset(1,:)','r-',[1:57]',newset(1,:)','g-')

end

