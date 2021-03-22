 % This function rotates single Coord into frame
function coords = convertCoord(Cf, point_set)
    c = (Cf\[point_set'; ones(1,size(point_set,1))])';
    coords = c(:,1:3);
end