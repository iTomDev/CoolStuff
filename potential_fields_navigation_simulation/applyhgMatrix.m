function [xyz2] = applyhgMatrixDOESNOTWORK(gfxmodel, hgtrans)
%UNTITLED Summary of this function goes here
%   Takes drawing object and the transform model. Extract the transform
%   matrix and apply to the x,y data from the drawing object then retrn the
%   x,y data. 

% DOES NOT WORK
% NEEDS MULTIDIEMSIONAL ARRAY OUTPUT OR USE FOR SINGLE OBJECTS ONLY

gfxmodel = gfxStaticObs;
hgtrans = hgStaticObs;

% loop trough graphics objects and extract the coords
%for gi=1:1:sel(size(gfxmodel),1,1)
%    xyz = [gfxmodel(gi).XData'; gfxmodel(gi).YData'];
%end

% loop through transform objects and extract the trasnform matrices
%xyz3 = zeros(100)';








% xyz = [gfxmodel(1).XData'; gfxmodel(1).YData'];
% 
% 
% 
% 
% 
% tmat = hgtrans;
% xyz = [gfxmodel.XData';
%        gfxmodel.YData';
%        zeros(1,(sel(size(gfxmodelXData,1,1))))]
end

