function j_hc = j_hc_R_sup(q)
    j_hc = numeric_jacobian(@output_func.hc_R_sup,q);
end