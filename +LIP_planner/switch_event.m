function [value,isterminal,direction] = switch_event(t,x,T)
    %value = u-2*x(1);
    value = x(3) - T;
    isterminal=1;
    direction=1;
end