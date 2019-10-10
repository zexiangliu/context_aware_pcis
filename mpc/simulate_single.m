function traj = simulate_single(varargin)
% Simulate the takeover scenario with one lead car and one ego car.

p = inputParser;
p.addParameter('time_horizon',15);
p.addParameter('plot_stuff',1);
p.addParameter('is_sp_on',1);
p.addParameter('is_print',0);
p.addParameter('pic_path','./pic/');
p.addParameter('lead_dyn',"agg");
p.addParameter('Iv',"bnd");
p.addParameter("x0", [30 0.1]');
p.addParameter("mpc","tailgate")
p.addParameter('CIS',Polyhedron());
p.addParameter('preXU',Polyhedron());
p.addParameter('xl', [22 25]');
p.addParameter('lead_lane',0);
p.parse(varargin{:});


time_horizon = p.Results.time_horizon;
plot_stuff = p.Results.plot_stuff;
is_sp_on = p.Results.is_sp_on; % is supervisor on
is_print = p.Results.is_print; % print the fig or not
pic_path = p.Results.pic_path;
lead_dyn = p.Results.lead_dyn;
lead_lane = p.Results.lead_lane;
xl = p.Results.xl;
N_l = length(lead_lane); % num of lead vehicles

mpc = p.Results.mpc;
Iv = p.Results.Iv;
x0 = p.Results.x0;
if is_print
    mkdir(pic_path);
end

con = constants_tri;

Iv_CIS = p.Results.CIS.copy();
Iv_preXU = p.Results.preXU.copy();

dim = Iv_CIS.Set(1).Dim;

for i = 1:N_l
    if ~containsPolyUnion(Iv_CIS,...
            [x0.*[1;-lead_lane(i)];xl(2*(i-1)+1)] + [0;1.8;0])
        error("bad initial point!")
    end
end

UnSafe = Polyhedron('UB',[inf inf con.h_min, inf],'LB',...
    -[inf inf con.h_min, inf]);

% disturbance
wx = zeros(time_horizon/con.dt+1,1);%2*con.dmax_ACC*rand(time_horizon/con.dt+1,1)-con.dmax_ACC;
wy = zeros(time_horizon/con.dt+1,1);%2*con.dmax_LK*rand(time_horizon/con.dt+1,1)-con.dmax_LK;
wL = zeros(time_horizon/con.dt+1,1);%2*con.dLmax*rand(time_horizon/con.dt+1,1)-con.dLmax;

% history
xEgo = x0;
xLd = xl;
xA1 = [x0;xl(1:2)];
X_list = [x0,zeros(2,time_horizon/con.dt)];
XL_list = [xl,zeros(2*N_l,time_horizon/con.dt)];
U_list = zeros(2,time_horizon/con.dt-1);
if is_sp_on
    UMPC_list = U_list;
end
US_list = [];
T_list = zeros(1,time_horizon/con.dt);

% simulate
if plot_stuff
    fig = figure('position',[100 100 1200 640]);
end
counter = 0;
t_detection = inf;
int_est = "bnd"; % intention estimation result
for t = 0:con.dt:time_horizon
    t
    counter = counter+1;
    
    %% put your controller here
    if is_sp_on
        if mpc == "tailgate"
            [u_c1, u_mpc, ~, U_f1] = mpc_supervisory(xEgo,xLd,lead_lane,Iv_CIS, Iv_preXU, con, t);
        elseif mpc == "takeover"
            [u_c1, u_mpc, ~, U_f1] = mpc_supervisory(xEgo,xLd,lead_lane, Iv_preXU, con);
        end
    else    
        if mpc == "tailgate"
            u_c1 = mpc_tailgate(xA1(1:3),xA1(4),con,t);    
        elseif mpc == "takeover"
            u_c1 = mpc_simple(xA1(1:3),xA1(4),con);    
        end
        U_f1 = get_input(Iv_preXU,xA1,dim);
    end
    
    u_c1 = u_c1(:,1);
%     u_c1 = mpc_supervisory(xA1,CIS_bnd, preXU_bnd, con);
    aEgoA1 = u_c1(1,1); 
    vyEgoA1 = u_c1(2,1);
    % make sure inputs satisfy limits
    aEgoA1 =  max(min(aEgoA1, con.umax_ACC), con.umin_ACC);
    vyEgoA1 =  max(min(vyEgoA1, con.umax_LK), con.umin_LK);
    u = [aEgoA1;vyEgoA1];
    
    %% visualize
    if plot_stuff
        %figure;
        xEgoA1 = -95 + sum(X_list(1,1:end-1))*con.dt;
        clf; 
        subplot(211)
        hold on    
        plot_road(xEgoA1,t);
        if is_sp_on
            plot_car2(xEgoA1,xEgo, xLd, lead_lane);
        else
            plot_car1(xEgoA1,xA1);
        end

        subplot(212);
        hold on
        title = "input of ego1";
        if is_sp_on
            plot_safe_input(U_f1,title,u_mpc, u);
        else
            plot_safe_input(U_f1,title,u);
        end
        drawnow;
        hold off
    end
    % make sure the car respects velocity and acceleration bounds
    w = [wx(counter),wy(counter),wL(counter)];
%%  state update
    for j = N_l:-1:1
        xA1 = [xEgo;xLd(2*j-1:2*j)];
        xA1 = update_dyn(xA1,u,w,UnSafe,con,lead_dyn(j));
        xLd(2*j-1:2*j) = xA1(3:4);
    end
    xEgo = xA1(1:2);
    X_list(:,counter+1) = xEgo;
    XL_list(:,counter+1) = xLd;
    U_list(:,counter) = u;
    if is_sp_on
        UMPC_list(:,counter) = u_mpc;
    end
    T_list(counter) = t;
    US_list = [US_list,U_f1];
    if plot_stuff && is_print
        print(fig,[pic_path,'frame',num2str(counter-1)],'-dpng');
    end
end
traj.Iv = Iv;
traj.lead_dyn = lead_dyn;
traj.sp_on = is_sp_on;
traj.X_list = X_list;
traj.U_list = U_list;
traj.US_list = US_list;
traj.T_list = T_list;
traj.t_detection = t_detection;
if is_sp_on
    traj.UMPC_list = UMPC_list;
end
