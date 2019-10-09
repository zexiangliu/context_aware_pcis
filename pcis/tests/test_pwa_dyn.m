clear all;
close all;
clc;

%% Initialize Library?

disp('Initialize Library? (Make sure ''../lib/'' is included)')
disp(' ')

%% Create Peicewise Affine Dynamics

n = 2;

%Creating System Matrices
A = [ -0.1 1 ; 0 -0.3 ];
B = eye(n);

E = [0; 1];

%Creating X-U Polyhedron
A_xu = [ 	zeros(2) eye(2);
			zeros(2) -eye(2) ];

u_min = -2;
u_max = 2;

b_xu = [ u_max*ones(2,1) ; -u_min*ones(2,1) ];

%Creating State-Dependent Disturbance Bounds
XW_V1 = [zeros(1,2) 1 -1];
XW_V2 = [zeros(1,2) -1 1];

%Creating Affine Dynamics
dyn1 = Dyn(	A,	zeros(n,1),	B,...
			Polyhedron('A',A_xu,'b',b_xu)) %, ...
			% [],[],Polyhedron(), ...
			% [],[],Polyhedron(), ...
			% [],Polyhedron(), ...
			% E, {XW_V1,XW_V2}); 

A2 = [-0.2 1; 0 -0.6];

dyn2 = Dyn(	A2,	zeros(n,1),	B,...
			Polyhedron('A',A_xu,'b',b_xu))%, ...
			% [],[],Polyhedron(), ...
			% [],[],Polyhedron(), ...
			% [],Polyhedron(), ...
			% E, {XW_V1,XW_V2}); 

disp('Finished Creating Dynamics.')

%Creating Domain

domain = Polyhedron('lb',[-2 -2],'ub',[2,2]);

figure;
plot(domain)
axis([-5 5 -5 5])

reg1 = domain.intersect(Polyhedron('A',[1 0],'b',0));
reg2 = domain.intersect(Polyhedron('A',[-1 0],'b',0));

figure;
plot(domain)
hold on;
plot(reg2,'color','blue')
axis([-5 5 -5 5])

reg_list = {reg1,reg2};

disp('Finished Creating Regions.')

% Create Piecewise Affine Dynamics
pwd1 = PwDyn(domain,reg_list,{dyn1,dyn2});
disp('Created Piecewise Affine Dynamics')
disp(' ')

%% Attempting Simple Pre Operation
disp('Attempting simple "pre" computation.')

X = Polyhedron('lb',[-0.5 -0.5],'ub',[0.5,0.5]); %Target Set.
N = 1; %Number of time-steps of backwards reachability.
eps0 = 0.01;

S = PolyUnion;
for i=1:pwd1.num_region
    new_poly = pwd1.reg_list{i}.intersect(pwd1.dyn_list{i}.pre_proj(X, eps0));
    % S = add1(S, new_poly);

    figure;
	hold on;
	plot(domain,'color','blue')
	plot(X,'color','magenta')
	plot(new_poly)
	axis([-5 5 -5 5])
end