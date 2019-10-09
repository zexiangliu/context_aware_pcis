function [preSets] = pre( varargin )

	% Usage:
	%	preSets = pre(pwd1 , X , rho)
	%	preSets = pre(pwd1 , X , rho , 'plot_stuff')
	% Inputs:
	%	pwd1	- The piecewise affine dynamics we are using.
	%	X 		- Target Set. The set for which we are calculating the pre.
	%
	% Outputs:
	%	preSets - An array of polyhedra. All possible polyhedra that could result in the target states X.

	%% Input Processing

	pwd1 = varargin{1};
	X = varargin{2};
	rho = varargin{3};

	plot_flag = false;

	if nargin >= 4
		for ind = 4:nargin
			switch varargin{ind}
				case 'plot_stuff'
					plot_flag = true;
                    figure;
				otherwise
					error(['Unrecognized input: ' varargin{ind} ])
			end
		end
    end

    if isa(X,'PolyUnion')
        %If the target set is a polyUnion, then we need to do the pre for
        %each element in its 'Sets'.
        temp_preSets = cell(X.Num,1);
        for ind_X = 1:X.Num
            temp_preSets{ind_X} = pwd1.pre(X.Set(ind_X),rho);
        end
        %After all is said and done, combine the polyUnions.
        all_sets = [];
        for ind_X = 1:length(temp_preSets)
            all_sets = [all_sets temp_preSets{ind_X}.Set ];
        end 
        preSets = PolyUnion(all_sets);
        
    elseif isa(X,'Polyhedron')

        % eps0 = 0.01;

        preSets = PolyUnion;
        for i=1:pwd1.num_region
            pre_from_dyn_i = pwd1.reg_list{i}.intersect(pwd1.dyn_list{i}.pre_proj(X, rho));
            % S = add1(S, new_poly);
%             pre_from_dyn_i.isEmptySet
            if plot_flag
                hold on;
                plot(pwd1.domain.projection([2 3]),'color','blue')
                plot(X.projection([2 3]),'color','magenta')
                plot(pre_from_dyn_i.projection([2 3]))
                axis([-5 5 -5 5])
            end

            preSets.add(pre_from_dyn_i);
        end
    else
        error('Target set is not of type Polyhedron or PolyUnion.');
    end

end