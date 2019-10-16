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
mptopt('lpsolver', 'GUROBI', 'qpsolver', 'GUROBI');

%% Create Safe Set and Small Invariant Set
h_max = Inf;
vl_max = Inf;
X1 = Polyhedron('UB', [con.v_max;   con.y_max;      h_max;      vl_max],...
                'LB', [con.v_min;   con.y_min;      con.h_min;     -vl_max]);
X2 = Polyhedron('UB', [con.v_max;   con.y_max;      h_max;     vl_max],...
                'LB', [con.v_min;   -con.y_min;     -h_max;    -vl_max]);
X3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min;    vl_max],...
                'LB', [con.v_min;   con.y_min;      -h_max;    -vl_max]);
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
rhoPre = 0; %1e-6;

X1 = Polyhedron('UB', [con.v_max;   con.y_max;      Inf],...
                'LB', [con.v_min;   con.y_min;      con.h_min]);
X2 = Polyhedron('UB', [con.v_max;   con.y_max;      Inf],...
                'LB', [con.v_min;   -con.y_min;     -Inf]);
X3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min],...
                'LB', [con.v_min;   con.y_min;      -Inf]);
S = PolyUnion([X1 X2 X3]);
C = X2;

Xr = dyn_conserv.stay_invariant(S, C, rhoPre, 1 );
 

%% Plotting

S1 = Polyhedron('UB', [con.v_max;   con.y_max;      50],...
                'LB', [con.v_min;   con.y_min;      con.h_min]);
S2 = Polyhedron('UB', [con.v_max;   con.y_max;      50],...
                'LB', [con.v_min;   -con.y_min;     -50]);
S3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min],...
                'LB', [con.v_min;   con.y_min;      -50]);

figure;
subplot(221); hold on;
temp_intersx = IntersectPolyUnion(Xr,PolyUnion([S1 S2 S3]));
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

