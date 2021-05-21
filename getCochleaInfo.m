% Run the gp_cochlea_estimate.m function and output a file that includes the
% path for the TRAC system to localize the magnet relative to 

% Add Necessary Folders
addpath('JRMPC');
addpath('DATA');
addpath('FUNCTIONS');

% ------PARAMETERS TO UPDATE:--------
feature_points_file = 'sample_data_file.dat'; % There should be 6 pts in this file
head_side = 0; % Left (0) or right (1) side of guinea pig head
output_file_name = 'gp_pos_bfield_1.txt'; % This is the file to output and be loaded by the Qt C++ code
%-----------------------------------

[co_est, ma_est] = gp_cochlea_estimate(feature_points_file,head_side);


% Generate Coordinate frame from the estimates
fp_data = load(feature_points_file);
RW_est = ((fp_data(4,:)-fp_data(5,:))/2)';
x_axis = ma_est;
y_axis = (RW_est-co_est)/norm(RW_est-co_est);
z_axis = cross(x_axis, y_axis);

R_cochlea = [x_axis y_axis z_axis];

% Transformation of cochlea in the camera frame:
c_T_co = [R_cochlea co_est;0 0 0 1];

%% Load cochlea Path file
% This loads the standard generic head path file which loads a specific
% field of b vectors for the omnimagnet to load and generate
[co_T, point_pos, point_field] = load_cochlea_path(head_side);

%% Transform position of cochlea and points
c_T = c_T_co * co_T;
points = (c_T_co * point_pos')';
field_at_points = (c_T_co * point_field')';

final_pts = points(:,1:3);
final_field = field_at_points(:,1:3);
%% Output file to be read by C++ program

fileID = fopen(strcat('COCHLEA_ESTIMATES/',output_file_name),'w');

% Write the transformation Matrix
for i=1:4
%     fprintf(fileID,'%3.5d %3.5d %3.5d %3.5d\n',c_T(i,:));
    fprintf(fileID,'%d %d %d %d\n',c_T(i,:));
end

% Write the field points and field vectors
for i=1:size(point_pos,1)
    fprintf(fileID,'%d %d %d %d %d %d\n',final_pts(i,1),final_pts(i,2),final_pts(i,3),final_field(i,1),final_field(i,2),final_field(i,3));
end

% Close the file
fclose(fileID);