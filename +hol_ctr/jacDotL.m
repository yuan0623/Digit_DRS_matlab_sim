function jj_c = jacDotL(q,dq)
    global dq_glb
    dq_glb = dq;
    jj_c = numeric_jacobian(@testingL,q)*dq;
end

function ans_ = testingL(q)
    global dq_glb
    ans_ = numeric_jacobian(@hol_ctr.left_holonomic_constraint,q)*dq_glb;
end