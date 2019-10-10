function delta = dual_delta(x0,u,UnSafe, con, intention)
    if x0(3) >= 0
        delta = -con.dLmax;
    else
        delta = -con.dLmin;
    end
%     A = [1-con.f1*con.dt 0 0 0
%         0 1 0 0
%         -con.dt 0 1 con.dt
%         0 0 0 1-con.f1*con.dt];
%     
%     B = [con.dt 0
%         0 con.dt
%         0 0
%         0 0];
%     if intention == "ann"
%         A_r = [zeros(3,4);con.K_ann*con.dt];
%     elseif intention == "cau"
%         A_r = [zeros(3,4);con.K_cau*con.dt];
%     end
%     
%     D = [0;0;0;con.dt];
%     
%     delta = sdpvar(1,1);
%     x_tp1 = (A+A_r)*x0 + B*u + D*delta;
%     p = sdpvar(4,1);
%     Constraints = [delta <= con.dLmax, ...
%         delta >= con.dLmin];
%     cost = (x_tp1-p)'*(x_tp1-p);
%     cost2 = x_tp1(3)^2;
%     ops = sdpsettings('solver','GUROBI','verbose',0);
%     optimize(Constraints,cost2,ops);
%     delta = value(delta);

    %% using quadprog
    
%     % decision varible [x_tp1;p;delta]
%     Aeq = [eye(4),zeros(4,4),-D];
%     beq = (A+A_r)*x0 + B*u;
%     
%     n_inq = length(UnSafe.b);
%     Ainq1 = zeros(n_inq,9);
%     Ainq1(:,5:8) = UnSafe.A;
%     binq1 = UnSafe.b;
%     Ainq2 = zeros(2,9);
%     Ainq2(:,end) = [1;-1];
%     binq2 = [con.dLmax;-con.dLmin];
%     Aineq = [Ainq1;Ainq2];
%     bineq = [binq1;binq2];
%     H_half = zeros(4,9);
%     H_half(:,1:4) = eye(4);
%     H_half(:,5:8) = -eye(4);
%     H = H_half'*H_half;
%     H(9,9) = 0.01;
%     delta = quadprog(H,zeros(9,1),Aineq,bineq,Aeq,beq);
%     delta = delta(end);
end
