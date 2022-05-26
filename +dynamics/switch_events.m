function [value,isterminal,direction] = switch_events(t,x,foot_index)
    q=x(1:30);
    if foot_index == -1
        swing_foot_pose = forward_kinematics.digit_left_foot_pose(q);

    elseif foot_index == 1
        swing_foot_pose = forward_kinematics.digit_right_foot_pose(q);
    end

    %if t>0.05
    %    value = swing_foot_pose(3);
        
    %else
    %    value = 10;
    %end
    value = swing_foot_pose(3);
    value;
        
    isterminal = 1;
    direction = -1;
end