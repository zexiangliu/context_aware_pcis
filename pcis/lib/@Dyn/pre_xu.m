function [ pre_XU ] = pre_xu(dyn, X, rho)
  % Compute set
  % {x : ∀ p ∃ u ∀ d, x_1(t+1) + Ball(rho) ⊆ X or x_2(t+1) + Ball(rho) ⊆ X}
  % using normal intersection/projection method
  % dynamics 1 and dynamcis 2 must have the same constraints on p, u and d.
  %
  % Reference: Petter Nilsson Ph.D. thesis (2017), Theorem 3.4

  if ~isa(dyn, 'Dyn')
    error('dyn must be an instance of Dyn');
  end

  if nargin < 3
    rho = 0;
  end

  if length(rho) == 1
    rho = rho * ones(dyn.nx,1);
  end

  % check if X is empty set
  if(isEmptySet(X))
      pre_XU = X;
      return;
  end
  
  Xb = X - Polyhedron('A', [eye(dyn.nx); -eye(dyn.nx)], 'b', repmat(rho,2,1));
  if(rho ~= 0)
    Xb.minHRep; % not sure it is necessary or not. 
  end

  DV = dyn.D.V;

  pre_XU = get_pre_proj(dyn,Xb,DV);
  pre_XU.minHRep;
end

function pre_proj = get_pre_proj(dyn,Xb,DV)
    A_mat_p = dyn.A;
    F_mat_p = dyn.F;

    Xd_A = zeros(0, dyn.nx+dyn.nu);
    Xd_b = zeros(0, 1);

    % For each d
    for id=1:max(1, size(DV,1))
      A_mat_pd = A_mat_p;
      F_mat_pd = F_mat_p;
      for jd=1:dyn.nd
        A_mat_pd = A_mat_pd + dyn.Ad{jd} * DV(id, jd);
        F_mat_pd = F_mat_pd + dyn.Fd{jd} * DV(id, jd);
      end
      if dyn.nw > 0
        % For each w
        for iw=1:length(dyn.XW_V)
          wext_x = dyn.XW_V{iw}(:, 1:dyn.nx);
          wext_w = dyn.XW_V{iw}(:, dyn.nx+dyn.np+1:end);
          Xd_A = [Xd_A; 
                  Xb.A*[A_mat_pd+dyn.Ew*wext_x dyn.B]];

          Xd_b = [Xd_b; 
                  Xb.b-Xb.A*(F_mat_pd+dyn.Ew*wext_w)];
        end
      else
        Xd_A = [Xd_A; 
                Xb.A*[A_mat_pd dyn.B]];
        Xd_b = [Xd_b; 
                Xb.b-Xb.A*F_mat_pd];
      end
    end
    pre_proj = Polyhedron('A', [Xd_A; dyn.XU.A], ...
                          'b', [Xd_b; dyn.XU.b]);
end
