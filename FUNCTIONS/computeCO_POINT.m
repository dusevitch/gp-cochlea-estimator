function point = computeCO_POINT(top_pt, bottom_pt, int_pt)
    d = (top_pt - bottom_pt) / norm(top_pt-bottom_pt);
    v = int_pt - bottom_pt;
    t = dot(v,d);
    point = bottom_pt + t * d;
end
