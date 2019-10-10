clc;clear all;close all;
time_horizon = 15;
plot_stuff = 1;
is_sp_on = 1; % is supervisor on
is_print = 1; % print the fig or not
pic_path = './test2/';
Iv = "bnd";

% -------------------
%        lead car in lane 1
% -------------------
%        lead car in lane 0
% -------------------
%        lead car in lane -1
% -------------------

% ego car states: [v_x, y], y is w.r.t. lane 0.
x0 = [30, 0.1]';
% lead car states: [h1, v_l1, h2, v_l2, h3, v_l3]
xl = [40, 25, 50, 25, 30, 25]';
% the lane of each lead car
lead_lane = [0, 1, -1];
lead_dyn = ["cau","cau","cau"];

sp_with_int = 0;
mpc = "tailgate";
load CIS_bnd_XU.mat
[exp3] = simulate_single('time_horizon',time_horizon,...
                         'plot_stuff',plot_stuff,...
                         'is_sp_on',is_sp_on,...
                         'is_print',is_print,...
                         'pic_path',pic_path,...
                         'lead_dyn',lead_dyn,...
                         'Iv',Iv,...
                         'x0',x0,...
                         'xl',xl,...
                         'mpc',mpc,...
                         'CIS', CIS_bnd,...
                         'preXU', preXU_bnd,...
                        'lead_lane',lead_lane);