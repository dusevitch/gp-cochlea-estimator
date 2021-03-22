function out =  inv_transformation_mat(trans_mat)
    R = trans_mat(1:3,1:3);
    t = trans_mat(1:3,4);
    out = [R' -R'*t; 0 0 0 1];
end