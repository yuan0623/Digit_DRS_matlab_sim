function j_hc = j_hc_L_sup(q)
    j_hc = numeric_jacobian(@output_func.hc_L_sup,q);
end