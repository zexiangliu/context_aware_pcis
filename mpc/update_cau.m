function [x_new, Iv] = update_cau(x,u,w,UnSafe,con)
% dynamics of aggressive driver
% update the state for one step
% inputs: x --- current state
%         u --- current input
%         w --- [wx,wy,wL] disturbances
%         UnSafe --- unsafe region
%         con --- output of tri_constants.m
    vEgoA2 = x(1);
    yEgoA2 = x(2);
    hA2 = x(3);
    vLeadA2 = x(4);
    aEgoA2 = u(1);
    vyEgoA2 = u(2);
    wx = w(1);
    wy = w(2);
    wL = w(3);
    Iv = "bnd";
     % make sure cautious car respects velocity and acceleration bounds
    if abs(hA2) < con.h_reaction
        deltaA2 = dual_delta(x, [aEgoA2;vyEgoA2], UnSafe, con, "cau");
        aLeadA2 = min(max(con.K_cau*x-con.K_cau(4)*con.vL_des+deltaA2, con.aL_min), con.aL_max)
        if nargout == 2
            Iv = intention_estimation(x, aLeadA2, con);
        end
    else
        aLeadA2 = min(max(-(vLeadA2-con.vL_des)/con.dt, con.aL_min), con.aL_max);
    end
    if vLeadA2 + aLeadA2 * con.dt > con.vL_max
        aLeadA2 = (con.vL_max - vLeadA2)/con.dt;
    elseif vLeadA2 + aLeadA2 * con.dt < con.vL_min
        aLeadA2 = (con.vL_min - vLeadA2)/con.dt;
    end
    % state updates
    hA2 = hA2 + (vLeadA2 - vEgoA2)*con.dt;
    vEgoA2 = vEgoA2 - con.f1*vEgoA2*con.dt + aEgoA2*con.dt + wx;
    yEgoA2 = yEgoA2 + vyEgoA2*con.dt + wy;
    vLeadA2 = vLeadA2 - con.f1*vLeadA2*con.dt + aLeadA2*con.dt + wL;
    x_new = [vEgoA2; yEgoA2; hA2; vLeadA2];
end
