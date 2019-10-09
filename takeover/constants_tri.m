function [con] = constants_tri()
  % Constants taken from the current TRI ACC model

  % sampling time
  con.dt = 0.1;

  % car dynamics 
  % longitudinal
  con.f1 = 3/36;
  con.f2 = 3/33.5;
  con.K_ann = [ 1 0 0 -1 ];
  con.K_cau = [0 -0.1 0.1 -0.01];
  con.k_cau = con.K_cau(4);
  con.vL_des = 30;
  con.K_des = [0 0 0 1];
  
  % lateral
  con.y_min = -0.9;
  con.y_max = 2.7;
  con.y_mid = 0.9;
%   con.y_lane = 3.6; % lane width
  
  % assumptions
  con.vL_min = 0;   % Lead car min velocity (m/s)
  con.vL_max = 33.5;  % Lead car max velocity (m/s)
  con.aL_min = -3; % m/s^2
  con.aL_max = 3;  % m/s^2
  
  % state/input/disturbance bounds
  con.v_min = 16;
  con.v_max = 36;
  con.umin_ACC = -3;
  con.umax_ACC = 3;
  con.umax_LK = 1.8; 
  con.umin_LK = -1.8;
  con.dmax_ACC = 0.05*con.umax_ACC;
  con.dmin_ACC = 0.05*con.umin_ACC;
  con.dmax_LK = 0.05*con.umax_LK;
  con.dmin_LK = 0.05*con.umin_LK;
  con.dLmax = 0.05*con.aL_max;
  con.dLmin = 0.05*con.aL_min;
    
  % specifications
  con.h_min = 10;   % minimum distance behind/ahead of lead car
  con.h_max = 300; % max distance behind/ahead of lead car (m)
  con.tau_min = 1.7; % minimum
  con.tau_des = 2.1;
  
  % piecewise affine dynamics
  con.vdz = (con.v_max+con.v_min)/2;
  con.beta1 = 0.135;
  con.h_reaction = 60;
  
end
