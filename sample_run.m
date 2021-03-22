% Example script to run gp_cochlea_estimate.m function

% Add Necessary Folders
addpath('JRMPC');
addpath('DATA');
addpath('FUNCTIONS');

% Run estimation function
[co_est, ma_est] = gp_cochlea_estimate('sample_data_file.dat',1);

% Display results
co_est
ma_est
