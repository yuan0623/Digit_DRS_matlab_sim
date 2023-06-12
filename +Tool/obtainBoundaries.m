function boundaries = obtainBoundaries(domain)
    if isempty(domain)
        boundaries = [];
    else
    initial_value = domain(1);
    boundaries = [initial_value,domain(2)];
    boundary_count = 1;
    for i = 2:length(domain)
        if domain(i)-domain(i-1)==1
            % not boundary
            boundaries(boundary_count,2) = domain(i);
        else
            boundary_count=boundary_count+1;
            boundaries(boundary_count,:) = [domain(i),domain(i)];
        end
    end
    end
end