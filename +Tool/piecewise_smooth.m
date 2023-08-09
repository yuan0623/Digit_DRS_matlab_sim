function V_global_filtered_smoothed = piecewise_smooth(V_global_filtered_clean, contact_indictor_global_filtered_clean)
    domain1 = [];
    domain2 = [];

    V_global_filtered_smoothed = V_global_filtered_clean;
    
    for i=1:length(contact_indictor_global_filtered_clean)

        if contact_indictor_global_filtered_clean(i) == 1
            domain1 = [domain1,i];
        elseif contact_indictor_global_filtered_clean(i) == -1
            domain2 = [domain2,i];
        end
    end
    %boundaries_domains = {};
    %boundaries_domains{1} = Tool.obtainBoundaries(domain1);
    %boundaries_domains{2} = Tool.obtainBoundaries(domain2);
    boundaries = [Tool.obtainBoundaries(domain1); Tool.obtainBoundaries(domain2)];
    for i = 1:length(boundaries)
        boundary = boundaries(i,:);
        V_global_filtered_smoothed(boundary(1):boundary(2)) = ...
            smooth(V_global_filtered_clean(boundary(1):boundary(2)),23);
    end


end