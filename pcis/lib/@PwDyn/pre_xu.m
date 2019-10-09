function [preXU] = pre_xu(pwd, X, rho)


    regions_xu = cell(pwd.num_region,1);
    for i = 1:pwd.num_region
         % lift the rigion to X-U Space
        tmp_reg = pwd.reg_list{i};
        regions_xu{i} = Polyhedron('H',[tmp_reg.A ...
                        zeros(size(tmp_reg.A,1),pwd.m)...
                        tmp_reg.b]);
    end
    
    preXU = [];
    for i = 1:pwd.num_region
            i
            tmp_P_xu = preUnion_xu(pwd.dyn_list{i},X,rho);
            tmp_P_xu = IntersectPolyUnion(regions_xu{i},tmp_P_xu);
            preXU = [preXU tmp_P_xu.Set];
    end
    
    preXU = PolyUnion(preXU);