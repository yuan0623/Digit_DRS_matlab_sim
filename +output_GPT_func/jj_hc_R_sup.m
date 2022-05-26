function jj_hc = jj_hc_R_sup(q,dq)
    jj_hc = numeric_jacobian(@testingR,q)*dq;
end

function ans_ = testingR(q)
    global dq_glb
    ans_ = numeric_jacobian(@output_GPT_func.hc_R_sup,q)*dq_glb;
end
