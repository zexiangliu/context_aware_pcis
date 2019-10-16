%pwd_pre_test1.m
%Description:
%	Testing how to create state-based disturbances by creating a simple 2d example.


%%%%%%%%%%%%%%%
%% Constants %%
%%%%%%%%%%%%%%%

n = 2;

eta_s = 5;
safe_set = Polyhedron(	'lb',-eta_s*ones(1,n), ...
						'ub',eta_s*ones(1,n));

eta_u = 1;
P_U = Polyhedron(	'lb',-eta_u*ones(1,1), ...
						'ub',eta_u*ones(1,1));

%% Define Dynamics

A = eye(2);
B = zeros(n,1);
F = B;
Bw = [1;0];

Ad = {zeros(n)};
% Fd = {[0 0 con.dt]' };
% Fd = {[0; 0; con.dt] };
Fd = {zeros(n,1) };

XU = safe_set * P_U;

%D = Polyhedron('lb',con.al_min,'ub',con.al_max);

dyn0 = Dyn(	A,F,B,XU,...
			{},{},Polyhedron(), ...
			{}, {}, Polyhedron(), ...
			zeros(0,1),{}, ...
			Bw, {[0,0,0],[1,0,1]});


%Parameters for pre
rho = 0.0;

%%%%%%%%%%%%%%%%
%% Algorithms %%
%%%%%%%%%%%%%%%%
target_set = Polyhedron('lb',-ones(1,n),'ub',ones(1,n));

pre_set = dyn0.pre(target_set,rho);

figure;
hold on;
safe_set.plot('Color','blue','Alpha',0.3)
target_set.plot('Color','red')
xlabel('$x_1$','Interpreter','latex')
ylabel('$x_2$','Interpreter','latex')

saveas(gcf,'target_plot','epsc')

figure;
hold on;
safe_set.plot('Color','blue','Alpha',0.3)
pre_set.plot('Color','red')
xlabel('$x_1$','Interpreter','latex')
ylabel('$x_2$','Interpreter','latex')

saveas(gcf,'pre_plot','epsc')

