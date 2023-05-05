function u_lateral_regulated = regulate_lateral_step(foot_index,u_lateral)
    is_regulated = false;
        if foot_index == -1 % right support
            if u_lateral < 0.1
                u_lateral_regulated = 0.1;
                is_regulated = true;
            elseif u_lateral > 0.45
                u_lateral_regulated = 0.45;
                is_regulated = true;
            else
                u_lateral_regulated = u_lateral;
            end
        elseif foot_index == 1 % left support
            if u_lateral > -0.1
                u_lateral_regulated = -0.1;
                is_regulated = true;
            elseif u_lateral < -0.45
                u_lateral_regulated = -0.45;
                is_regulated = true;
            else
                u_lateral_regulated = u_lateral;
            end
        end
        if is_regulated == true
            fprintf("step length is regulated")
        end
end