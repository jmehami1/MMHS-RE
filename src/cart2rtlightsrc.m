function [r,ptsTheta] = cart2rtlightsrc(x, y, z)
% Given 3D cartestian points w.r.t to a non-isotropic point source, find
% the radius (distance of point from origin) and theta about its principal
% direction (assumed to be along the z-dir). The model is assumed to be
% radially symmetric, so all angles are positive. 
% INPUT:
%       ptsLightSrc - 3D cartesian points w.r.t to point source coordinate
%           frame [3 x number of points]
% OUTPUTS:
%       r - radius of points from point source
%       ptsTheta - Radian angle of points w.r.t to principal direction

%Author: Jasprabhjit Mehami, 13446277

%radius of pointS from source
r = sqrt(x.^2 + y.^2 + z.^2);

ptsLightSrc = [x;y;z];

%create matrix of z-direction vectors
zDirVec = zeros(size(ptsLightSrc));
zDirVec(3,:) = 1;

%angle of vector to point about principal direction of light source
ptsTheta = acos(dot(ptsLightSrc, zDirVec) ./ r);

end

