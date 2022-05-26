function jj_hc = jj_hc_L_sup(q,dq)
    jj_hc = numeric_jacobian(@testingL,q)*dq;
end

function ans_ = testingL(q)
    global dq_glb
    ans_ = numeric_jacobian(@output_func.hc_L_sup,q)*dq_glb;
end

