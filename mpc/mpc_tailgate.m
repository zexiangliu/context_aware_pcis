function U = mpc_tailgate(x0, v_l, con, t, side)
% implement MPC for simple dynamics here 
%   inputs: x0 --- initial state
%           v_l --- measurement of lead car speed
%           con --- constraints and parameters            
    h = 50; % time horizon you want to predict
    n = 3; % num of states
    m = 2; % num of inputs
    
    % states: x = [v_ex, y_e, h]
    % inputs: u = [a_ex, v_ey]
    % dynamics: x(k+1) = Ax(k) + Bu(k) + F
    A = [1-con.f1*con.dt 0 0
         0 1 0
         -con.dt 0 1];
    B = [con.dt 0; 
         0 con.dt;
         0 0];
    F = [0; 0; v_l*con.dt];
    
    
    % the decision vector is [x(1), x(2), x(3), ..., x(h),
    %                          u(0),u(1),...,u(h-1)] (h*(m+n) x 1)
    num = h*(m+n);
    % index of x(k)_i = (k-1)*n + i
    % index of x(k) = (k-1)*n+1:k*n
    % index of u(k)_i = h*n + k*m + i
    % index of u(k) = h*n + k*m+1:h*n + (k+1)*m
    ind_x = @(k) (k-1)*n+1:k*n;
    ind_x_i = @(k,i) (k-1)*n + i;
    ind_u = @(k) h*n + k*m+1:h*n + (k+1)*m;
%     ind_u_i = @(k,i) h*n + k*m+1:h*n + (k+1)*m;
    ind_x_i_all = @(i) i:n:h*n;
%     ind_u_i_all = @(i) (h*n+i):m:num;
    % === Equality Constraints ===
    Aeq = [];
    beq = [];
    % dynamics constraints
        % initial step: x(1) = Ax(0) + Bu(0) + F
        % x(1) - Bu(0) = Ax(0) + F
    Aeq1 = zeros(n,num);
    Aeq1(:,ind_x(1)) = eye(n);
    Aeq1(:,ind_u(0)) = -B;
    beq1 = A*x0 + F;
    Aeq = [Aeq;Aeq1];
    beq = [beq;beq1];
    
    % k step: x(k) = Ax(k-1) + Bu(k-1) + F
    % x(k)-Bu(k-1)-Ax(k-1) = F
    for i = 2:h
        Aeqi = zeros(n,num);
        Aeqi(:,ind_x(i)) = eye(n);
        Aeqi(:,ind_x(i-1)) = -A;
        Aeqi(:,ind_u(i-1)) = -B;
        beqi = F;
        Aeq = [Aeq;Aeqi];
        beq = [beq;beqi];
    end
    
    % === Inequality constraints === 
    Aineq = [];
    bineq = [];
    
    for i = 1:h
        % con.v_min <= v_ex(i) <= con.v_max
        Aineq_x = zeros(4,num);
        Aineq_x(1,ind_x_i(i,1)) = 1;
        Aineq_x(2,ind_x_i(i,2)) = 1;
        Aineq_x(3,ind_x_i(i,1)) = -1;
        Aineq_x(4,ind_x_i(i,2)) = -1;
        bineq_x = [con.v_max; con.y_max;-con.v_min;-con.y_min];
        % con.umin <= u(i-1) <= con.umax
        Aineq_u = zeros(4,num);
        Aineq_u(1:2,ind_u(i-1)) = eye(2);
        Aineq_u(3:4,ind_u(i-1)) = -eye(2);
        bineq_u = [con.umax_ACC;
                   con.umax_LK;
                   -con.umin_ACC;
                   -con.umin_LK];
        Aineq = [Aineq;Aineq_x;Aineq_u];
        bineq = [bineq;bineq_x;bineq_u];
    end
    
    % === cost functions ===
    
    % indices of all x_1(t), x_2(t), x_3(t)
    idx1 = ind_x_i_all(1);
    idx2 = ind_x_i_all(2);
    idx3 = ind_x_i_all(3);
    
    % mode_0: a*(h-h_des)^2+b*u^2
    a = 10;
    b = 5;
    h_des = 10.5;
    H0 = zeros(num,num);
    H0(idx3,idx3) = eye(length(idx3))*a;
    H0(ind_x_i(h,n)+1:end,ind_x_i(h,n)+1:end) = eye(num-ind_x_i(h,n))*b;
    f0 = zeros(num,1);
    f0(idx3) = -a*h_des;
    
    % mode 1: a*(v_ex-v_desired)^2+b*u^2
    v_d = 30;
    y_d = 0;
    a = 10;
    b = 0;
    c = 5;
    H1 = zeros(num,num);
    H1(idx1,idx1) = eye(length(idx1))*a;
    f1 = zeros(num,1);
    f1(idx1) = -a*v_d;
    H1(idx2,idx2) = eye(length(idx2))*c;
    f1(idx2) = -c*y_d;
    H1(ind_x_i(h,n)+1:end,ind_x_i(h,n)+1:end) = eye(num-ind_x_i(h,n))*b;
    
    % mode 2: a*(v_ex-v_desired)^2+b*u^2 + c*(y-y_desired)^2 + d*(h-h_des)^2
    v_d = con.v_max;
    y_d = side*1.8;
    a = 10;
    b = 1;
    c = 20;
    d = 0;
    H2 = zeros(num,num);
    h_des2 = -10;
    H2(idx1,idx1) = eye(length(idx1))*a;
    f2 = zeros(num,1);
    f2(idx1) = -a*v_d;
    H2(idx2,idx2) = eye(length(idx2))*c;
    f2(idx2) = -c*y_d;
    H2(idx3,idx3) = eye(length(idx3))*d;
    f2(idx3) = -d*h_des2;
    H2(ind_x_i(h,n)+1:end,ind_x_i(h,n)+1:end) = eye(num-ind_x_i(h,n))*b;

    % mode 3: a*(v_ex-v_desired)^2+b*u^2 + c*(y-y_desired)^2 - d*h
    v_d = 30;
    y_d = 0;
    a = 10;
    b = 1;
    c = 20;
    d = 15;
    H3 = zeros(num,num);
    H3(idx1,idx1) = eye(length(idx1))*a;
    f3 = zeros(num,1);
    f3(idx1) = -a*v_d;
    H3(idx2,idx2) = eye(length(idx2))*c;
    f3(idx2) = -c*y_d;
    f3(idx3) = -d;
    H3(ind_x_i(h,n)+1:end,ind_x_i(h,n)+1:end) = eye(num-ind_x_i(h,n))*b;
    
    % quadprog
    h_act = 20; % the h to start takeover
    if t<= 3 || x0(3) >= h_act
        [X,~,flag] = quadprog(H0,f0,Aineq,bineq,Aeq,beq);
%     elseif x0(3) >= h_act
%         [X,~,flag] = quadprog(H1,f1,Aineq,bineq,Aeq,beq);
    elseif x0(3) < h_act && x0(3) >= h_des2
        [X,~,flag] = quadprog(H2,f2,Aineq,bineq,Aeq,beq);
    else
        [X,~,flag] = quadprog(H3,f3,Aineq,bineq,Aeq,beq);
    end
    
    if flag == 1
        U = reshape(X(h*n + 1:end),[m,h]);
    else
%         keyboard();
        warning("no solution!");
        U = reshape(X(h*n + 1:end),[m,h]);
    end
    
end
