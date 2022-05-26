function dq_plus=resetmap(x,foot_index)

    q=x(1:30);
    dq=x(31:60);
    [D,~,~] = dynamics.dynamic_matrix_digit(x,foot_index);
    if foot_index == -1
        hol_ctr_jacobian = numeric_jacobian(@hol_ctr.left_holonomic_constraint,q);
    elseif foot_index == 1
        hol_ctr_jacobian = numeric_jacobian(@hol_ctr.right_holonomic_constraint,q);
    end
    b=[D*dq';0;0;0;0;0;0;0;0;0;0];
    A=[D,-hol_ctr_jacobian';
       hol_ctr_jacobian,zeros(10,10)];
    dq_plus=A\b;
    dq_plus=dq_plus(1:30);
    
    
end