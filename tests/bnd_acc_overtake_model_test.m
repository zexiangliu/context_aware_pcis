%bnd_acc_overtake_model_test.m
%Description:
%	Attempts to verify that the dynamics defined by get_dyn_bdd_acc.m are actually valid.



%%%%%%%%%%%%%%%
%% Constants %%
%%%%%%%%%%%%%%%

dyn_ut = get_dyn_bdd_acc();

%%%%%%%%%%%
%% Tests %%
%%%%%%%%%%%

disp('Does dyn_ut''s dynamics define a domain and region set that properly define a partition?')
is_this_a_partition(dyn_ut.domain,dyn_ut.reg_list)