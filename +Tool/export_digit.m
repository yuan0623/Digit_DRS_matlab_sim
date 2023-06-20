function export_digit(digit_robot)
    for i = 1:length(digit_robot.Mmat)
        M = digit_robot.Mmat{i};
        export(M,'gen_v2')
    end

end