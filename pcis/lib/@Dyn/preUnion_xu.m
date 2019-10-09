function X0 = preUnion_xu(dyn, X, rho)
% pre_int_ux for the input is a PolyUnion

    if nargin < 3
        rho = 0;
    end

    if isa(X, 'PolyUnion')	
        X0 = PolyUnion;
        for i=1:X.Num
            new_poly = pre_xu(dyn, X.Set(i), rho);
            X0.add(new_poly);
        end
    else
        X0 = pre_xu(dyn, X, rho);
    end
end