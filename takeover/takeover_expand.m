% takeover_expand.m
% Passing Scenario for TRI

% System matrices,
%   state = [v_e,x]
%           [y_e  ]
%           [h    ]
%
%   where - v_e,x is the ego car's velocity in the x direction
%         - y_e is the ego car's lateral displacement (where left is
%         positive, right is negative.)
%         - h is the distance between the lead car's back bumper and the
%         ego car's front bumber (positive indicates ego is following the
%         lead car)
%         - v_L,x is the lead car's velocity in the longitudinal or x
%         direction, which is a disturbance term.

%% Constants %%
clear;close all;clc;
con = constants_tri();
% Get Dynamics
dyn_conserv = get_dyn_bdd_vel2();
pwd_bdd_acc = get_dyn_bdd_acc();
mptopt('lpsolver', 'GUROBI', 'qpsolver', 'GUROBI');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Choose Dynamics to use %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Select the dimension of overtake model that you want
n = 4

%% Create Safe Set and Small Invariant Set
h_max = Inf;
vl_max = Inf;
switch n
	case 3
		%Use the 3dimensional overtake model
		X1 = Polyhedron('UB', [con.v_max;   con.y_max;      h_max],...
		                'LB', [con.v_min;   con.y_min;      con.h_min]);
		X2 = Polyhedron('UB', [con.v_max;   con.y_max;      h_max],...
		                'LB', [con.v_min;   -con.y_min;     -h_max]);
		X3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min],...
		                'LB', [con.v_min;   con.y_min;      -h_max]);
		
	case 4
		%Use the 4 dimensional overtake model
		X1 = Polyhedron('UB', [con.v_max;   con.y_max;      con.h_max;      con.vL_max],...
                		'LB', [con.v_min;   con.y_min;      con.h_min;  	con.vL_min]);
		X2 = Polyhedron('UB', [con.v_max;   con.y_max;      con.h_max;     	con.vL_max],...
		                'LB', [con.v_min;   -con.y_min;     -con.h_max;    	con.vL_min]);
		X3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min; 	con.vL_max],...
		                'LB', [con.v_min;   con.y_min;      -con.h_max;    	con.vL_min]);
	otherwise
		error('Unexpected dimension of dynamics given.')
end

% Safe set 
S = PolyUnion([X1 X2 X3]); 
% figure;clf;hold on
% for s=1:S.Num
%     plot(S.Set(s).projection([2 3]));
% end
% set(gca,'Xdir','reverse','Ydir','reverse');
% xlabel('y');
% ylabel('h');

% cinv set
C = X2;

% cinv set
% load CIS_bnd.mat
% C = lift_inv(CIS_bnd);

% reach
rhoPre = 1e-6; %1e-6;

switch n
case 3
	Xr = dyn_conserv.stay_invariant(S,C,rhoPre,1);
case 4
	Xr = pwd_bdd_acc.expand(S, C, rhoPre , 'debug' ,'max_iter' , 30,'plot_stuff',0);
end	

%% Plotting

switch n
	case 3
		S1 = Polyhedron('UB', [con.v_max;   con.y_max;      50],...
		                'LB', [con.v_min;   con.y_min;      con.h_min]);
		S2 = Polyhedron('UB', [con.v_max;   con.y_max;      50],...
		                'LB', [con.v_min;   -con.y_min;     -50]);
		S3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min],...
		                'LB', [con.v_min;   con.y_min;      -50]);

		set_to_plot = Xr;

	otherwise
		S1 = Polyhedron('UB', [con.v_max;   con.y_max;      50; 		con.vL_max],...
		                'LB', [con.v_min;   con.y_min;      con.h_min; 	con.vL_min]);
		S2 = Polyhedron('UB', [con.v_max;   con.y_max;      50; 		con.vL_max],...
		                'LB', [con.v_min;   -con.y_min;     -50; 		con.vL_min]);
		S3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min; con.vL_max],...
		                'LB', [con.v_min;   con.y_min;      -50; 		con.vL_min]);
		%polyh_arr = [];
		for poly_idx = 1:Xr.Num
			polyh_arr(poly_idx) = Xr.Set(poly_idx).slice([4],16);
		end

		set_to_plot = PolyUnion(polyh_arr);
end


figure;
subplot(221); hold on;
temp_intersx = IntersectPolyUnion(set_to_plot,PolyUnion([S1.slice([4],16) S2.slice([4],16) S3.slice([4],16)]) );
plot(temp_intersx.slice([1],25),'color','red')
set(gca,'Xdir','reverse','Ydir','reverse')
axis([-1 3 -50 50]);
xlabel('ye'); ylabel('h');
title('vEgo = 25 m/s')

subplot(222); hold on;
plot(temp_intersx.slice([1],30),'color','red')
set(gca,'Xdir','reverse','Ydir','reverse')
axis([-1 3 -50 50]);
xlabel('ye'); ylabel('h');
title('vEgo = 30 m/s')

subplot(223); hold on;
plot(temp_intersx.slice([1],16),'color','red')
set(gca,'Xdir','reverse','Ydir','reverse')
axis([-1 3 -50 50]);
xlabel('ye'); ylabel('h');
title('vEgo = 16 m/s')

subplot(224); hold on;
plot(temp_intersx.slice([1],20),'color','red')
set(gca,'Xdir','reverse','Ydir','reverse')
axis([-1 3 -50 50]);
xlabel('ye'); ylabel('h');
title('vEgo = 20 m/s')

