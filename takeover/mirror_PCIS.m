function W_r = mirror_PCIS(W)
% flip the PCIS in dimension y
if isa(W,'Polyhedron')
    W_r = mirror_Polytope(W);
elseif isa(W,'PolyUnion')
    W_r = PolyUnion();
    for i = 1:W.Num
        W_r.add( mirror_Polytope(W.Set(i)));
    end
else
    error("Input has to be a polyhedron or a polyunion!");
end
end

function W_r = mirror_Polytope(W)
   H_r = W.H;
   H_r(:,2) = -H_r(:,2);
   W_r = Polyhedron('H', H_r);
end