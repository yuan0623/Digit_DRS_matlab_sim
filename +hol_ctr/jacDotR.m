function jj_c = jacDotR(q,dq)
    global dq_glb
    dq_glb = dq;
    jj_c = numeric_jacobian(@testingR,q)*dq;
end

function ans_ = testingR(q)
    global dq_glb
    ans_ = numeric_jacobian(@hol_ctr.right_holonomic_constraint,q)*dq_glb;
end