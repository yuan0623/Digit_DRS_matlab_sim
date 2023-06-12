function domainIndicator(contact_indictor,t_vec,y_min,y_max)
    domain1 = [];
    domain2 = [];
    domain3 = [];
    domain4 = [];
    domain5 = [];
    domain6 = [];
    for i=1:length(contact_indictor)

        if contact_indictor(i) == 1
            domain1 = [domain1,i];
        elseif contact_indictor(i) == -1
            domain2 = [domain2,i];
        end
    end
    boundaries_domains = {};
    boundaries_domains{1} = Tool.obtainBoundaries(domain1);
    boundaries_domains{2} = Tool.obtainBoundaries(domain2);
    
    colors = [[1 0 0];[0 0.4470 0.7410];[0.3010 0.7450 0.9330]];
    %colors = ['r','g','b','c','m','y'];
    for i = 1:length(boundaries_domains)
        boundaries = boundaries_domains{i};
        if isempty(boundaries)
        else
            for j = 1:length(boundaries(:,1))
                X = [t_vec(boundaries(j,1)),t_vec(boundaries(j,1)),t_vec(boundaries(j,2)),t_vec(boundaries(j,2))];
                Y = [y_min,y_max,y_max,y_min];
                p = patch(X,Y,colors(i,:));
                %p.FaceColor = colors(i);
                alpha(0.13)
                p.EdgeColor = 'none';
            end
        end
    end
end
