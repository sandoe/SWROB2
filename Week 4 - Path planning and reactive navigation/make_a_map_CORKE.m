
clear

% CORKE make a matrix/image
p = makemap;

% check matrix/image
imagesc(p), colorbar

% Navigation / Robotics systems toolbox MATLAB - occupancy map object
ocmap = binaryOccupancyMap(p);
show(ocmap)

% save ocmap
save("ocmap.mat", "ocmap")

