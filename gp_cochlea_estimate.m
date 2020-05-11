% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GP_COCHLEA_ESTIMATE - Estimate the cochlear origin and modiolar axis of a guinea pig
%	Using data from 11 other guinea pigs and the JRMPC algorithm, the cochlea location and 
%   modiolar axis orientation can be found for points selected for a Dunkin-Hartley Guinea pig
%
% Syntax:  [co_pos_vec, ma_orient_vec] = gp_cochlea_estimate(input_gp_points, head_side)
%
% Inputs:
%    input_gp_points - 10x3 input array of feature coordinates of guinea pig you wish to 
%                      estimate cochlea information for. 
%    head_side - Left (0) or right (1) side of guinea pig head
%
% Outputs:
%    co_pos_vec - 3x1 Cochlea-Origin Position Vector Estimate
%    ma_orient_vec - 3x1 Modiolar-Axis Unit Vector Estimate
%
% Example:
%    [co, ma] = gp_cochlea_est('sample_gp_features.dat',0)
%
% Other m-files required: jrmpc.m and associated files (see Readme.txt), 
% Subfunctions: none
%
% Author: David Usevitch
% email: david.usevitch@utah.edu
% Website: http://www.telerobotics.utah.edu
% Mar 2020; Last revision: 18-Mar-2020

function [co_pos_vec, ma_orient_vec] = gp_cochlea_estimate(input_gp_points, head_side)
    
    % FOR REFERENCE ONLY:
    % features = [CB,FPI,FPS,LSC,MA,OI,OS,RWFF,RWCF,ISC,ASC,ST,CA,IN,ZA,CO]
    % validation_set = [2,3,4,5,6,7,8,9,12,15];
    
    train_set = [1,2,3,4,5,6,7,8,9,10,11,12,15,16];
    
    % Load Data
    ccL = load('combined_coords_L_CO.mat');
    ccR = load('combined_coords_R_CO.mat');

    all_points_L = ccL.combined_coords_L;
    all_points_R = ccR.combined_coords_R;
    
    % Generate all points based on the training_set    
    PTS_L = permute(num2cell(permute(all_points_L(train_set,:,:),[2,1,3]),[1,2]), [3,2,1]);
    PTS_R = permute(num2cell(permute(all_points_R(train_set,:,:),[2,1,3]),[1,2]), [3,2,1]);
    
    input_gp_data = input_gp_points; % Should load into an 8x3 dim vector
    maxNumIter = 100;
    num_sets = 12;

    % Use the proper point set based on the right or left side of gp head
    if head_side == 0
        PTS_set = PTS_L; % left side
        all_points_set = all_points_L;
    else
        PTS_set = PTS_R; % right side
        all_points_set = all_points_R;
    end
    
    % Initialize/Load Variables
    CB_index = 1; % These indices are based on the full set (from the 11 pts)
    CA_index = 13;
    CO_index = 16;
    
    FPI_ind = 1; % These indices are based on the input set
    FPS_ind = 2;
    ST_ind = 9;
    
    FPI = input_gp_data(FPI_ind,:);
    FPS = input_gp_data(FPS_ind,:);
    ST = input_gp_data(ST_ind,:);

    % Solve for coordinate point (center of coordinate frame).
    syms x y z t;
    AB = (FPS-FPI);
    eqns = [x == FPI(1) + t*AB(1), y == FPI(2) + t*AB(2), z == FPI(3) + t*AB(3) , dot(AB, [x,y,z]) == dot(AB,ST)];
    solx = solve(eqns,[x y z t]);
    D = [eval(solx.x) eval(solx.y) eval(solx.z)]; % Intersecting Point (CENTER OF COORD FRAME).  This is the perpendicular intersection of the modiolar axis and closest point to the median foot plate point.
    
    % Generate coordinate Frame as Homogeneous Transformation:
    x = AB/norm(AB);
    y = (ST-D)/norm(ST-D);
    z = cross(x,y);
    
    C_fo = [[x' y' z'] D';0 0 0 1]; % Coordinate Frame in homogeneous coords
    
    % Put input points into the footplate frame
    coords = convertCoord(C_fo, input_gp_points);
    
    % Add the input point to the point cloud set - extra ST point
    PTS_set{num_sets,1} = coords';
    
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

    % Get the mean of all points excluding the input point set
    mean_transformed_pts = mean(transformed_pts,3);
    
    % CO position and MA computation in JRMPC frame
    CO = mean_transformed_pts(CO_index,:);
    CA = mean_transformed_pts(CA_index,:);
    CB = mean_transformed_pts(CB_index,:);
    
    Cf_jf = [R{num_sets} t{num_sets};0 0 0 1]; % From the footplate (f) frame to the (j) frame
    Cf_jo = Cf_jf*C_fo; % From the original frame (o) to the jrmpc (j) frame
    
    % Get cochlea-origin position and modiolar-axis from jrmpc to original frame
    mean_transformed_orig_coords = convertCoord(inv(Cf_jo), mean_transformed_pts);
    
    % Transformed errors back into the original input frame
    CB_pos = mean_transformed_orig_coords(CB_index,:)';
    CA_pos = mean_transformed_orig_coords(CA_index,:)';
    CO_pos = mean_transformed_orig_coords(CO_index,:)';
    
    % Output estimates
    co_pos_vec = CO_pos;
    ma_orient_vec = CA_pos - CB_pos;

end