function [preSets,preXU] = pre_int_cvx(pwd1, pwd2, X, rho, regions, dyns_id, isParallel,isOnlyXU)

	% Usage:
	%	preSets = pre_int(pwd1 , pwd2, X , rho)
	% Inputs:
	%	pwd1,pwd2	- The piecewise affine dynamics we are using.
	%	X 		- Target Set. The set for which we are calculating the pre.
	%
	% Outputs:
	%	preSets - An array of polyhedra. All possible polyhedra that could result in the target states X.

	%% Input Processing
	
    if(nargin <= 4 || isempty(regions) || isempty(dyns_id))
        regions = cell(pwd1.num_region*pwd2.num_region,1);
        dyns_id = cell(pwd1.num_region*pwd2.num_region,1);
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
    else
        num_reg = length(regions);
    end
    
    ifpreXU = false;
    if(nargout > 1)
        ifpreXU = true;
        regions_xu = cell(pwd1.num_region*pwd2.num_region,1);
        for i = 1:num_reg
             % lift the rigion to X-U Space
            tmp_reg = regions{i};
            regions_xu{i} = Polyhedron('H',[tmp_reg.A ...
                            zeros(size(tmp_reg.A,1),pwd1.m)...
                            tmp_reg.b]);
        end
    end

    if(nargin <= 6)
        isParallel = false;
        isOnlyXU = false;
    elseif(nargin <= 7)
        isOnlyXU = false;
    end
    
    MaxNum = 30 + X.Num;
    
    if(isParallel)
        
        P_xu1 = cell(pwd1.num_region,1);
        P_xu2 = cell(pwd2.num_region,1);
        parfor i = 1:pwd1.num_region
            tmp_P_xu = preUnion_xu(pwd1.dyn_list{i},X,rho);
            P_xu1{i} = tmp_P_xu;
        end
        
        parfor i = 1:pwd2.num_region
            tmp_P_xu = preUnion_xu(pwd2.dyn_list{i},X,rho);
            P_xu2{i} = tmp_P_xu;
        end
        
        new_poly = cell(num_reg,1);
        if ifpreXU
            new_poly_xu = cell(num_reg,1);
        else
            new_poly_xu = cell(num_reg,1);
            regions_xu = cell(num_reg,1);
        end
        parfor i=1:num_reg
            dyn_id1 = dyns_id{i}(1);
            dyn_id2 = dyns_id{i}(2);
            P_inter = IntersectPolyUnion(P_xu1{dyn_id1},P_xu2{dyn_id2});
            if ~isOnlyXU
                new_poly{i} = IntersectPolyUnion(regions{i},...
                    projectionPolyUnion(P_inter,1:pwd1.n));
            end
            if ifpreXU
                new_poly_xu{i} = IntersectPolyUnion(regions_xu{i},P_inter);
            end
        end
        
        preSets = [];
        preXU = [];
        for i = 1:num_reg
            preSets = [preSets new_poly{i}.Set];
            if ifpreXU
                preXU = [preXU new_poly_xu.Set];
            end
        end
        
        preSets = PolyUnion(preSets);
        if ifpreXU
            preXU = PolyUnion(preXU);
        end
%         % reduce if too many polyhedron
%         if(preSets.Num >= MaxNum)
%             preSets.reduce;
%         end
    else
        P_xu1 = cell(pwd1.num_region,1);
        P_xu2 = cell(pwd2.num_region,1);
        for i = 1:pwd1.num_region
            tmp_P_xu = merge_cvx(preUnion_xu(pwd1.dyn_list{i},X,rho));
            P_xu1{i} = tmp_P_xu;
        end
        
        for i = 1:pwd2.num_region
            tmp_P_xu = merge_cvx(preUnion_xu(pwd2.dyn_list{i},X,rho));
            P_xu2{i} = tmp_P_xu;
        end
        
        preSets = PolyUnion;
        preXU = PolyUnion;
        
        for i=1:num_reg
            disp(i);
            dyn_id1 = dyns_id{i}(1);
            dyn_id2 = dyns_id{i}(2);
            P_inter = merge_cvx(IntersectPolyUnion(P_xu1{dyn_id1},P_xu2{dyn_id2}));
            
            if ~isOnlyXU
                new_poly = IntersectPolyUnion(regions{i},...
                    merge_cvx(projectionPolyUnion(P_inter,1:pwd1.n)));
                
                if new_poly.Num > 0
                    new_poly = merge_cvx(new_poly);
                    preSets = PolyUnion([preSets.Set new_poly.Set]);
                end
            end

                        
            if ifpreXU
                new_poly_xu = IntersectPolyUnion(regions_xu{i},P_inter);
                new_poly_xu = merge_cvx(new_poly_xu);
                if new_poly_xu.Num > 0
                    preXU = PolyUnion([preXU.Set new_poly_xu.Set]);
                end
            end
%             % reduce if too many polyhedron
%             if(preSets.Num >= MaxNum)
%                 preSets.reduce;
%                 MaxNum = max(MaxNum,preSets.Num*2);
%             end
        end
        
% %         preSets = PolyUnion(preSets);
%         if ifpreXU
%             preXU = PolyUnion(preXU);
%         end
        
        % reduce if too many polyhedron
%         if(preSets.Num >= MaxNum)
%             preSets.reduce;
%         end
    end
end