function [M,c,B] = dynamic_matrix_digit(x,foot_index)
    q = x(1:30);
    dq = x(31:60);
    M = zeros(30,30);
    for i = 1:26
       file_name = "Mmat"+string(i)+"_digit(q)"; 
       M = M+eval(file_name); 
    end
    c = -Ge_vec_digit(q,dq);
    %% full actuation
    
    B =    [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1;];
        
    %% under actuation
    %{
    if foot_index == 1
        B =    [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %1
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %2
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %3
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %4
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %5
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %6
                1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %7
                0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %8
                0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %9
                0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %10
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %11
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %12
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %13
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %14
                0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;  %15
                0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0;  %16
                0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;  %17
                0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;  %18
                0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0;  %19
                0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0;  %20
                0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0;  %21
                0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0;  %22
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %23
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %24
                0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0;  %25
                0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0;  %26
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0;  %27
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0;  %28
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0;  %29
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1;];%30
    elseif foot_index == -1
        B =    [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %1
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %2
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %3
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %4
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %5
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %6
                1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %7
                0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %8
                0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %9
                0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %10
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %11
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %12
                0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;  %13
                0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0;  %14
                0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;  %15
                0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;  %16
                0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0;  %17
                0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0;  %18
                0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0;  %19
                0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0;  %20
                0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0;  %21
                0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0;  %22
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %23
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %24
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %25
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  %26
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0;  %27
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0;  %28
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0;  %29
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1;];%30   
    end
    %}
end