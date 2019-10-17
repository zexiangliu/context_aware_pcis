function [ pwd_out ] = get_dyn_bdd_acc(varargin)
% get_dyn_bdd_acc.m
%Description:
%	System with the states
%			[ v_e ]
%		x = [ y_e ]
%			[  h  ]
%			[ v_L ]
%	and which obeys both velocity AND acceleration constraints.
%
%Usage:
%	dyn4d = get_dyn_bdd_acc()
%	dyn4d = get_dyn_bdd_acc('disturb_type',1)

%% Input Processing
arg_idx = 1;
while arg_idx < nargin
	switch varargin{arg_idx}
		case 'disturb_type'
			disturb_type = varargin{arg_idx+1};
			arg_idx = arg_idx + 1;
		otherwise
			error('Unexpected input to the function get_dyn_bdd_acc().')
	end
end

if ~exist('disturb_type')
	disturb_type = 1; %State dependent disturbances.
end

%% Constants

con = constants_tri();
n_u = 2;
n_x = 4;

%%%%%%%%%%%%%%%
%% Algorithm %%
%%%%%%%%%%%%%%%

%% Define Regions
%Region 1, No Possibility for violation of vL bounds
H_r1 = [ 	0,0,0,-1;
			0,0,0,1 ];
h_r1 = [ 	-con.vL_min+con.aL_min*con.dt;
			con.vL_max-con.aL_max*con.dt ];

r1 = Polyhedron('A',H_r1,'b',h_r1);

%Region 2, Violation of vL maximum bounds
H_r2 = -[ 0,0,0,1 ];
h_r2 = [-con.vL_max+con.aL_max*con.dt];

r2 = Polyhedron('A',H_r2,'b',h_r2);

%Region 3, violation of vL minimum bounds
H_r3 = [ 0,0,0,1 ];
h_r3 = [con.vL_min - con.aL_min*con.dt];

r3 = Polyhedron('A',H_r3,'b',h_r3);

switch disturb_type
	case 1
		A = eye(n_x);
		A(3,1) = -con.dt;
		A(3,4) = con.dt;

		B = [ eye(n_u)*con.dt ; zeros(n_x-n_u,n_u) ];
		Bw = [ 0;0;0; con.dt ];
		F = zeros(n_x,1);

		XU = Polyhedron('A',[zeros(n_u,n_x) eye(n_u) ; zeros(n_u,n_x) -eye(n_u) ], ...
		                'b',[con.umax_ACC ; con.umax_LK ; -con.umin_ACC ; -con.umin_LK ]);

		%% Defining the regions for Piece-wise dynamics
		
		%Region 1
		CH1 = {[zeros(1,n_x),con.aL_min],[zeros(1,n_x),con.aL_max]};
		dyn1 = Dyn(	A,F,B,XU,...
					{},{},Polyhedron(), ...
					{},{}, Polyhedron(), ...
					zeros(0,1),{}, ...
					Bw, CH1);

		%Region 2
		CH2 = {[zeros(1,n_x),con.aL_min],[0,0,0,-1/con.dt,(1/con.dt)*con.vL_max]};
		dyn2 = Dyn(	A,F,B,XU,...
					{},{},Polyhedron(), ...
					{},{}, Polyhedron(), ...
					zeros(0,1),{}, ...
					Bw, CH2);

		%Region 3
		CH3 = {[0,0,0,-1/con.dt,(1/con.dt)*con.vL_min],[zeros(1,n_x),con.aL_max]};
		dyn3 = Dyn(	A,F,B,XU,...
					{},{},Polyhedron(), ...
					{},{}, Polyhedron(), ...
					zeros(0,1),{}, ...
					Bw, CH3);

		%% Define PwDyn object
		dom = Polyhedron('lb',[con.v_min,con.y_min,-con.h_max,con.vL_min],'ub',[con.v_max,con.y_max,con.h_max,con.vL_max] );
		pwd_out = PwDyn(dom, ...
						{ r1.intersect(dom) , r2.intersect(dom) , r3.intersect(dom) }, ...
						{ dyn1 , dyn2 , dyn3 } );

	case 2
		error('State-independent disturbance part is not defined yet...')

	otherwise
		body
end

