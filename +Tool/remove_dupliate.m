function [vec_a_removed,vec_b_removed] =  remove_dupliate(vec_a,vec_b)
    epislon = 0.001;
    a1 = vec_a(1);
    vec_a_removed = [];
    vec_b_removed = [];
    for i = 2:length(vec_a)
        delta_ = vec_a(i)-a1;
        if delta_>epislon
            vec_a_removed = [vec_a_removed,vec_a(i)];
            vec_b_removed = [vec_b_removed,vec_b(:,i)];
            a1 = vec_a(i);
        end
    end
end