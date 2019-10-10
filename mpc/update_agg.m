function [x_new, Iv] = update_agg(x,u,w,UnSafe,con)
% dynamics of aggressive driver
% update the state for one step
% inputs: x --- current state
%         u --- current input
%         w --- [wx,wy,wL] disturbances
%         UnSafe --- unsafe region
%         con --- output of tri_constants.m
    vEgoA1 = x(1);
    yEgoA1 = x(2);
    hA1 = x(3);
    vLeadA1 = x(4);
    aEgoA1 = u(1);
    vyEgoA1 = u(2);
    wx = w(1);
    wy = w(2);
    wL = w(3);
    Iv = "bnd";
    % make sure the car respects velocity and acceleration bounds
    if abs(hA1) < con.h_reaction
        deltaA1 = dual_delta(x, [aEgoA1;vyEgoA1], UnSafe, con, "ann");
%         aLeadA1 = min(max(con.K_cau*x-con.K_cau(4)*con.vL_des+deltaA1, con.aL_min), con.aL_max);
        aLeadA1 = min(max(con.K_ann*x+deltaA1, con.aL_min), con.aL_max);

        if nargout == 2
            Iv = intention_estimation(x, aLeadA1, con);
        end
    else
        aLeadA1 = min(max(-(vLeadA1-con.vL_des)/con.dt, con.aL_min), con.aL_max);
    end
    if vLeadA1 + aLeadA1 * con.dt > con.vL_max
        aLeadA1 = (con.vL_max - vLeadA1)/con.dt;
    elseif vLeadA1 + aLeadA1 * con.dt < con.vL_min
        aLeadA1 = (con.vL_min - vLeadA1)/con.dt;
    end
    % state updates
    hA1 = hA1 + (vLeadA1 - vEgoA1)*con.dt;
    vEgoA1 = vEgoA1 - con.f1*vEgoA1*con.dt + aEgoA1*con.dt + wx;
    yEgoA1 = yEgoA1 + vyEgoA1*con.dt + wy;
    vLeadA1 = vLeadA1 - con.f1*vLeadA1*con.dt + aLeadA1*con.dt + wL;
    x_new = [vEgoA1; yEgoA1; hA1; vLeadA1];

end
