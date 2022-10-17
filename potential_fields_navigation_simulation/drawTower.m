function [] = drawTower(desx, desy)
 
x = [0 0 2 3 5 5 3 2 0];
y = [2 3 5 5 3 2 0 0 2];
% find the offset from the edge to central point
offstx = (max(x)-min(x))/2;
offsty = (max(y)-min(y))/2;
offstx = desx+offstx;
offsty = desy-offsty;
for i=1:1:sel(size(x),1,2)
    sx(:,i) = [offstx offsty]'+[x(i) y(i)]';
end
plot(sx(1,:), sx(2,:), 'r-', 'LineWidth', 1);

end

