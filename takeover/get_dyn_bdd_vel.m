function [ dyn0 ] = get_dyn_bdd_vel()
% get_dyn_bdd_vel.m

%% Constants

con = constants_tri();
n_u = 2;
n_x = 3;

%% Algorithm

A = [	1-con.f1*con.dt 0 0;
		0 				1 0;
		-con.dt 		0 1];

B = [ eye(n_u)*con.dt ; zeros(1,n_u) ];
Bw = [ eye(n_x)*con.dt ];

F = zeros(n_x,1);

XU = Polyhedron('A',[zeros(n_u,n_x) eye(n_u) ; zeros(n_u,n_x) -eye(n_u) ], ...
                'b',[con.umax_ACC ; con.umax_LK ; -con.umin_ACC ; -con.umin_LK ]);

Ad = {zeros(3),zeros(3),zeros(3)};
Fd = { Bw(:,1) , Bw(:,2) , Bw(:,3) };

D = Polyhedron('lb',[con.dmin_ACC,con.dmin_LK,con.vL_min],'ub',[con.dmax_ACC,con.dmax_LK,con.vL_max]);

dyn0 = Dyn(	A,F,B,XU,...
			{},{},Polyhedron(), ...
			Ad, Fd, D);