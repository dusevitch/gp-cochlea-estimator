% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GP_COCHLEA_ESTIMATE - Estimate the cochlear origin and modiolar axis of a guinea pig
%	Using data from 11 other guinea pigs and the JRMPC algorithm, the cochlea location and 
%   modiolar axis orientation can be found for points selected for a Dunkin-Hartley Guinea pig
%
% Syntax:  [co_pos_vec, ma_orient_vec] = gp_cochlea_estimate(input_gp_points, head_side)
%
% Inputs:
%    TOUCH_DATA_FILE - 10x3 input array of feature coordinates of guinea pig you wish to 
%                      estimate cochlea information for. 
%    head_side - Left (0) or right (1) side of guinea pig head
%
% Outputs:
%    co_pos_vec - 3x1 Cochlea-Origin Position Vector Estimate
%    ma_orient_vec - 3x1 Modiolar-Axis Unit Vector Estimate
%
% Example:
%    [co, ma] = gp_cochlea_estimate('sample_gp_features.dat',0)
%
% Other m-files required: jrmpc.m and associated files in folders (see Readme.txt), 
% Subfunctions: none
%
% Author: David Usevitch
% email: david.usevitch@utah.edu
% Website: http://www.telerobotics.utah.edu
% Mar 2021; Last revision: 22-Mar-2021

function [co_pos_vec, ma_orient_vec] = gp_cochlea_estimate(TOUCH_DATA_FILE, head_side)

    % Winning training and test set indices based on training CT examples
    train_set = [1,3,4,7,8,9,10,12,13,14,15,16,17,18,19,20];
    test_set = [3,8,10,12,14,16];

    % For loading files for Algorithm Matching
    ccL = load('combined_coords_method2_morepts_final_co_correct_L_CO.mat');
    ccR = load('combined_coords_method2_morepts_final_co_correct_R_CO.mat');
    all_points_L = ccL.cc_L;
    all_points_R = ccR.cc_R;
    %----------------------------------------------------------------
    %% STEP 1) LOAD DATA
    if head_side == 0
    all_points_set = all_points_L;
    else
    all_points_set = all_points_R;
    end

    data_touched_features = load(TOUCH_DATA_FILE);

    % Generate all points based on the training_set    
    PTS_set = permute(num2cell(permute(all_points_set(train_set,:,:),[2,1,3]),[1,2]), [3,2,1]);

    %% STEP 2) Estimate the MA, CO from the touched points (data_touched_features) using our algorithm found previously

    %------------------------ 2a) Generally Align Touched Points --------------------------
    % indexes here refer to the 16 input points from the touched set (data_touched)
    RWCF_touch = data_touched_features(4,:);
    RWFF_touch = data_touched_features(5,:);
    ST_touch = data_touched_features(6,:);

    % Solve for coordinate point (center of coordinate frame).
    CO_rot_est = computeCO_POINT(RWCF_touch,RWFF_touch,ST_touch); % This is the CO Point we are using for aligning the frame before the alg matching
    if head_side == 0
    % LEFT SIDE
    x = (RWCF_touch-RWFF_touch)/norm(RWCF_touch-RWFF_touch);
    y = (ST_touch-CO_rot_est)/norm(ST_touch-CO_rot_est);
    z = cross(x,y);
    else
    % Right SIDE
    x = -(RWCF_touch-RWFF_touch)/norm(RWCF_touch-RWFF_touch);
    y = (ST_touch-CO_rot_est)/norm(ST_touch-CO_rot_est);
    z = cross(x,y);
    end
    trans_matrix_touch = [[x' y' z'] [CO_rot_est-(ST_touch-CO_rot_est)]';0 0 0 1]; %Coordinate Frame
    inv_trans_matrix_touch = inv_transformation_mat(trans_matrix_touch);
    data_touched_trans_frame_pts = [convertCoord(trans_matrix_touch, data_touched_features)];

    %------------------------ 2b) Run the JRMPC ALGORITHM --------------------------

    maxNumIter = 60;
    num_sets = 12;

    % Add the input point to the point cloud set - extra ST point
    PTS_set{num_sets,1} = data_touched_trans_frame_pts';

    % Get the mean of the point clouds
    PTS_mean = zeros(3,num_sets);
    for n = 1:num_sets
    PTS_mean(:,n) = mean(PTS_set{n},2);
    end

    % Run the JRMPC algorithm
    [R,t] = jrmpc(PTS_set, PTS_mean,'maxNumIter',maxNumIter,'gamma',0.05, 'epsilon', 1e-9);

    % Transform All points (Do not include the input set)
    transformed_pts = zeros(size(all_points_set));
    for p = 1:num_sets-1
    transformed_pts(:,:,p) = (R{p}*all_points_set(:,:,p)' + t{p})';
    end

    % Test Plot to see if points are all generally aligned after the JRMPC Algorithm
    all_transformed_pts = zeros(length(test_set), size(all_points_set,2), size(all_points_set,3)+1);
    for n = 1:num_sets-1
    tpts = transformed_pts(:,:,n);
    all_transformed_pts(:,:,n) = tpts(test_set,:);
    end

    JRMPC_trans_touched_pts = (R{num_sets}*data_touched_trans_frame_pts' + t{num_sets})';
    all_transformed_pts(:,:,num_sets) = JRMPC_trans_touched_pts;

    %%

    % --------------------- 2c) POINT MEAN AND CO/MA ESTIMATE FROM ALGORITHM ----------------------
    % Get the mean of all points excluding the input point set
    mean_transformed_pts = mean(transformed_pts,3);

    CO_index = 20;
    CB_index = 1;
    TC_index = 17;

    % CO position and MA computation in JRMPC frame
    CO = mean_transformed_pts(CO_index,:);
    TC = mean_transformed_pts(TC_index,:); % Use the TC from what was touched
    CB = mean_transformed_pts(CB_index,:);


    j_Cf_r = [R{num_sets} t{num_sets};0 0 0 1]; % From the RW (r) frame to the (j) frame
    r_Cf_o = inv_trans_matrix_touch;
    j_Cf_o = j_Cf_r*r_Cf_o; % From the original frame (o) to the jrmpc (j) frame

    % inv of r_Cf_o
    o_Cf_r = [r_Cf_o(1:3,1:3)' -r_Cf_o(1:3,1:3)'*r_Cf_o(1:3,4) ; 0 0 0 1];
    r_Cf_j = [j_Cf_r(1:3,1:3)' -j_Cf_r(1:3,1:3)'*j_Cf_r(1:3,4) ; 0 0 0 1];

    % inv of j_Cf_o
    o_Cf_j = [j_Cf_o(1:3,1:3)' -j_Cf_o(1:3,1:3)'*j_Cf_o(1:3,4) ; 0 0 0 1];

    % Get cochlea-origin position and modiolar-axis from jrmpc to original frame
    mean_transformed_orig_coords = convertCoord(j_Cf_o, mean_transformed_pts);

    % Transformed errors back into the original input frame
    CB_pos = mean_transformed_orig_coords(CB_index,:)';
    TC_pos = mean_transformed_orig_coords(TC_index,:)';
    CO_pos = mean_transformed_orig_coords(CO_index,:)';

    % Output estimates
    co_pos_vec = CO_pos;
    ma_orient_vec = (TC_pos - CB_pos)/norm(TC_pos - CB_pos); % Normalize the vector output

end