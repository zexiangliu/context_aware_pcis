function U = get_input(inv_set,x, dim)
    if nargin == 2
        dim = 4;
    else
        x = x(1:dim);
    end
    U = inv_set.slice(1:dim,x);
    
end