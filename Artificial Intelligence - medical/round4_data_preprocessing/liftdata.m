function [ newdata] = liftdata( datain )
%LIFTDATA 
% Lifts the data
% Algorithm: Marcos Rodruiguez
% Minor mods: Thomas Pile

% lifting the data up
%Compensate for time decay in bacteria collection patterns
%The functions fits a straight line to each row of DATA,
%then pivots on the first curve and raises each row in turn
%until the straight line is horizontal
data = datain;

%find how many rows and how many columns in the data
[nrows,ncols] = size(data);

%initialize the function's return value with input data
newdata = data; 

%First produces list of polynomial 1-st order fits per row of data, keep in variable Grad
for r = 1:nrows
    Grad(r,:) = polyfit(1:ncols,newdata(r,:),1);
end
	
%correct newdata(r,c) for decay in time
for r = 1:nrows
    for c=1:ncols
        newdata(r,c) = newdata(r,c) - Grad(r,1)*(c);
    end
end

end

