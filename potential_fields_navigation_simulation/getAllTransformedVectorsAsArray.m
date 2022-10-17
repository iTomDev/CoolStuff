function [xyz3] = getAllTransformedVectorsAsArray(gfxmodel, hgtrans)
%GETALLTRANSFORMEDVECTORSASARRAY Summary of this function goes here
%   This is basically a bodge to go through all graphics objects, apply
%   their respective transforms from hgtransform, then put the whole lot
%   into a big array and use it generate potential fields. The alternative
%   is to use a multidimensional array but its more complexity than really
%   required for this.
% returns x,y,z,x,y,z

%gfxmodel = gfxStaticObs;
%hgtrans = hgStaticObs;

%xyz2 = zeros(sel(size(hgtrans),1,1)*10, 6); % rough est of number of pairs of vectors
xyz2 = zeros(size(hgtrans,1)*10, 6); % rough est of number of pairs of vectors

% loop through the transforms
sofar = 0;
%for ti=1:1:sel(size(hgtrans),1,1)
for ti=1:1:size(hgtrans,1)
    % load the coords for that model
    xyz = [gfxmodel(ti).XData'; gfxmodel(ti).YData'];
    % loop through the coords
    %for gi=1:1:(sel(size(xyz),1,2)-1)
    for gi=1:1:size(xyz,2)-1
        % perform transform on the coord pair
        %xyz2(gi,1:6) = hgtrans(ti).Matrix*[xyz(1,gi) xyz(2,gi), 0, 1]';
        xyza =(hgtrans(ti).Matrix*[xyz(1,gi) xyz(2,gi), 0, 1]')';
        xyzb =(hgtrans(ti).Matrix*[xyz(1,gi+1) xyz(2,gi+1), 0, 1]')';
        xyz2(gi+sofar,1:6) = [xyza(1:3) xyzb(1:3)];
    end
    sofar = sofar+gi;
end

xyz3 = xyz2(1:sofar,:); % copy out data from temp erray, it removes the zero records at the end

sofar;
xyz3;

end

