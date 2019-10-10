function [u_s, u ,U, U_f] = mpc_supervisory(xEgo, xLd, lead_lane, C, C_XU, con, t)
% the mpc controller under supervisory
% inputs: x0 --- current state
%         v_l --- lead car speed
%         C --- inv set in X space
%         C_XU --- inv set in XU space
% outputs: u --- current input
%          U --- input sequence gotten by MPC
%          U_f --- polyunion, feasible inputs
    
%% MPC
    % go left if y >= 0, otherwise go right
    if xEgo(2) >= 0
        side = 1;
    else
        side = -1;
    end
    if nargin == 6
        U = mpc_simple([xEgo;xLd(1)],xLd(2), con, side);
    elseif nargin == 7
        U = mpc_tailgate([xEgo;xLd(1)],xLd(2), con, t, side);
    end
    u = U(:,1);
    
%% Barrier
    % we need to identify what lane the ego car is on:
    if xEgo(2) >= -con.y_mid && xEgo(2)<= con.y_mid
        ego_lane = 0;
    elseif xEgo(2) >= con.y_mid && xEgo(2)<= 3*con.y_mid
        ego_lane = 1;
    elseif xEgo(2) <= -con.y_mid && xEgo(2)>= -3*con.y_mid
        ego_lane = -1;
    end
    
    dim = C.Set(1).Dim;
    if nargout == 4
        U_f = PolyUnion();
        flag_Uf = 1;
    else
        flag_Uf = 0;
    end
    u_s = u;
    
    U_f = PolyUnion();
    for i = 1:length(lead_lane)
        % if the ego and lead are not in the same or adjacent lanes, skip
        if abs(lead_lane(i)-ego_lane) > 1
            continue;
        end
        
        % construct the relative coordinates
        u2_flip = 1;
        if lead_lane(i) > ego_lane
            x_r = [xEgo(1);lead_lane(i)*2*con.y_mid - xEgo(2);xLd(2*i-1);xLd(2*i)];
            u2_flip = -1;
        else
            x_r = [xEgo(1);xEgo(2) - lead_lane(i)*2*con.y_mid;xLd(2*i-1);xLd(2*i)];
        end
        
        % if lead_lane(i) == ego_lane
        if lead_lane(i) == ego_lane
            x_r2 = x_r;
            x_r2(2) = - x_r2(2);
            u2_flip2 = -1;
            flag1 = containsPolyUnion(C,x_r(1:dim));
            flag2 = containsPolyUnion(C,x_r2(1:dim));
            if ~flag1 && ~flag2
                warning("not in inv set.");
                U_f = Polyhedron();
                break;
            else 
                if flag1 
                    U_f1 = get_input(C_XU, x_r, dim);
                else
                    U_f1 = PolyUnion();
                end
                if flag2
                    U_f2 = get_input(C_XU, x_r2, dim);
                else
                    U_f2 = PolyUnion();
                end
                
                U_f1 = flipU(U_f1, u2_flip);
                U_f2 = flipU(U_f2, u2_flip2);
                U_f.add([U_f1.Set,U_f2.Set]);
            end
        else
            if ~containsPolyUnion(C,x_r(1:dim))
                warning("not in inv set.");
                U_f = Polyhedron();
                break;
            else
                U_f1 = get_input(C_XU, x_r, dim);
                U_f1 = flipU(U_f1, u2_flip);
                U_f = IntersectPolyUnion(U_f,U_f1);
%                 U_f.reduce();
            end
        end
    end
    
    if ~containsPolyUnion(U_f, u)
        if U_f.Num >= 20
            U_f.reduce();
        end
        proj_dist = inf;
        u_proj = [];
        for p = 1:U_f.Num
            pp = U_f.Set(p);
            if pp.isEmptySet
                continue
            else
                H = eye(2);
                f = -u;
                [u_tmp, fval, exitflag] = quadprog(H,f,pp.A,pp.b);
                if exitflag == 1
                    if fval < proj_dist
                        proj_dist = fval;
                        u_proj = u_tmp;
                    end
                end
            end
        end
        if ~isempty(u_proj)
            u_s = u_proj;
        end
    end
end

function U_f = flipU(U,u2_flip)
% flip the second input of the admissible input set
    if u2_flip == 1
        U_f = U;
    else
        U_f = PolyUnion;
        for i = 1:U.Num
            if ~U.Set(i).isEmptySet
                H = U.Set(i).H;
                H(:,2) = u2_flip*H(:,2);
                U_f.add(Polyhedron('H',H));
            end
        end
    end
end