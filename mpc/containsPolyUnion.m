function bool = containsPolyUnion(V,x)
    
    for i = 1:V.Num
        if V.Set(i).contains(x)
            bool = true;
            return;
        end
    end
    bool = false;
end