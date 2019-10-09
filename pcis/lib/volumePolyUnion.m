function volume = volumePolyUnion(V)
    volume = 0;
    if isa(V,'Polyhedron')
        volume = V.volume;
        return;
    end
    
    for i = 1:V.Num
        volume = volume + V.Set(i).volume;
    end
end