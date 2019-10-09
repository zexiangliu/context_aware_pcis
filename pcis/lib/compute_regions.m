function [regions, dyns_id] = compute_regions(pwd1,pwd2)
    
    num_reg = 0;
    
    for i = 1:pwd1.num_region
        for j = 1:pwd2.num_region
            tmp_inter = intersect(pwd1.reg_list{i},pwd2.reg_list{j});
            if ~isEmptySet(tmp_inter) && tmp_inter.volume > 0
                num_reg = num_reg + 1;
                tmp_inter.minHRep;
                regions{num_reg} = tmp_inter;
                dyns_id{num_reg} = [i,j];
            end
        end
    end
    
end