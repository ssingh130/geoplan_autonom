function d=haversine(lat,long)

% HAVERSINE     Computes the  haversine (great circle) distance in metres
% between successive points on the surface of the Earth. These points are
% specified as vectors of latitudes and longitudes.
%
%This was written to compute the distance between
%
%   Examples
%       haversine([53.463056 53.483056],[-2.291389 -2.200278]) returns
%       6.4270e+03
%
%   Inputs
%       Two vectors of latitudes and longitudes expressed in decimal
%       degrees. Each vector must contain at least two elements.
%
%   Notes
%       This function was written to process data imported from a GPS
%       logger used to record mountain bike journeys around a course.

long=deg2rad(long);
lat=deg2rad(lat);

dlat=diff(lat);
dlong=diff(long);

a=sin(dlat/2).^2+cos(lat(1:end-1)).*cos(lat(2:end)).*sin(dlong/2).^2;
c=2*atan2(sqrt(a),sqrt(1-a));

R=6371000; %in metres
d=R.*c;