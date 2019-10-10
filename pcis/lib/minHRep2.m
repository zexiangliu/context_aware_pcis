% alternative function of PolyHedron.minHRep
% Emil Klintberg, Magnus Nilsson, Lars Johannesson Mårdh, Ankit Gupta. "A primal
% active-set minimal representation algorithm for polytopes with
% application to invariant-set calculations".

function C_min = minHRep2(C,checked_ind)
    if nargin == 1
        checked_ind = [];
    end
    
%     method = 1; % use the new minimal hyperplane technique
    method = 2; % use the MPT3 minHRep;
%     method = 1; % debuging mode

    if method == 1
%         try
            if any(C.b < 0)
                z = mean(C.V,1)';
                [A,b,ind_min] = indicate_nonredundant_halfplanes(C.A,C.b,checked_ind,z);
            else
                [A,b,ind_min] = indicate_nonredundant_halfplanes(C.A,C.b,checked_ind);
            end

%             C_min = Polyhedron('A',A,'b',b);
            C_min = Polyhedron('A',C.A(ind_min,:),'b',C.b(ind_min));
%         catch ME
%             disp("minHRep2 fails with the following error:");
%             disp(ME.message);
%             C_min = C.minHRep;
%         end
    elseif method == 2
        C_min = C.minHRep;
    elseif method == 3
        C_min = C;
    elseif method == 4
        if any(C.b < 0)
            z = mean(C.V,1)';
            [A,b,ind_min] = indicate_nonredundant_halfplanes(C.A,C.b,checked_ind,z);
        else
            [A,b,ind_min] = indicate_nonredundant_halfplanes(C.A,C.b,checked_ind);
        end

%             C_min = Polyhedron('A',A,'b',b);
        C_min = Polyhedron('A',C.A(ind_min,:),'b',C.b(ind_min));
        
        C_min_tmp = C.minHRep;
        
        if ~(C_min_tmp == C_min) %size(C_min_tmp.H,1) ~= size(C_min.H,1)
            disp("catch mismatch in minHRep!");
            keyboard(); 
        end
    end
        
end