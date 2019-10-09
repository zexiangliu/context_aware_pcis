function [ is_partition ] = is_this_a_partition( dyn )
	%is_this_a_partition.m
	%
	%Inputs:
	%	D - The set that we believe is being partitioned.
	%		Polyhedron
	%	
	%	reg_set - The collection of sets that we want to test to see
	%			  if it partitions D.
	%			
	%
	%Assumptions:
	%	
	%
	%Outputs
	%	is_partition - True if reg_set is a partition of D.
	%				   Boolean
	%


	%% Constants
    D = dyn.domain;
    reg_set = dyn.reg_list;
	%% Check to See if every element of reg_set is "Disjoint"
	all_are_disjoint = true;
	for reg_ind = 1:(length(reg_set)-1)
		for reg2_ind = (reg_ind+1):length(reg_set)
			temp_isect = reg_set{reg_ind}.intersect(reg_set{reg2_ind});
			if temp_isect.volume ~= 0
				all_are_disjoint = false;
				warning('Not all regions in the set are disjoint.')
			end
		end
	end

	%% Check to See if D is a subset of U (reg_set)
	temp_D = D.copy();
	for reg_ind = 1:length(reg_set)
		temp_D = temp_D \ reg_set{reg_ind};
	end
	reg_set_contains_D = (temp_D.volume == 0);

	if ~reg_set_contains_D
		warning('D is not a subset of reg_set.')
	end

	%% Check if U (reg_set) is a subset of D
	D_contains_reg_set = true;
	for reg_ind = 1:length(reg_set)
		temp_minus = (reg_set{reg_ind} \ D );
		if (temp_minus.volume ~= 0)
			D_contains_reg_set = false;
		end
	end

	if ~D_contains_reg_set
		warning('reg_set is not a subset of D.')
	end

	is_partition = all(all_are_disjoint) && all(reg_set_contains_D) && all(D_contains_reg_set);

end

