function [ fixed ] = fixdata( broken )

% use a linear interpolation to get a curve
% if you get an error "index exceeds dimensions", delete the variable "fit"
y = broken;
f = fit([1:37]',y,'cubicinterp','Exclude',y<1);
% apply fix
for(i=1:37)
    if y(i)==0 
        fixed(i) = f(i);
    else
        fixed(i) = y(i);
    end
end

end

