function proj = projectionPolyUnion(P,dims)
    proj = PolyUnion;
    for i = 1:P.Num
        proj.add(minHRep(projection2(P.Set(i),dims)));
%         proj.add(P.Set(i).projection(dims));
    end

end