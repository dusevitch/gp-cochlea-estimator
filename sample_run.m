% Example script to run gp_cochlea_estimate.m function

% Add Folders
addpath('JRMPC');
addpath('DATA');
addpath('FUNCTIONS');

% Load example points to test:
input_pts = load('input_data_set.dat'); % CHANGE NAME HERE FOR DIFFERENT FILE

l_r = 1; % left or right 0-left 1-right side

% Run estimation function
[co_est, ma_est] = gp_cochlea_estimate(input_pts,l_r);