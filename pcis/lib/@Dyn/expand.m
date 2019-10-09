function [Ct] = expand(dyn, C0, Safe, show_plot, verbose, maxIter)
% expand algorithm for Dyn: compute the maximum invariant set in C from C0.
% input C0 must be a control invariant set (seed).
%  
%  Pre(C) ⊂ C
% 
% which is a sufficient condition for controlled invariance
%
% Reference: Rungger, M., & Tabuada, P. (2017). Computing Robust 
% Controlled Invariant Sets of Linear Systems. IEEE Trans. on 
% Automatic Control, dx.doi.org/10.1109/TAC.2017.2672859

  if nargin < 4
    show_plot = 0;
  end

  if nargin < 5
    verbose = 0;
  end

  C = Polyhedron;
  Ct = C0;
  iter = 0;

  tic;

  if show_plot
    figure; clf
  end

  while not (Ct <= C)
    C = Ct;

    if show_plot
      plot(C, 'alpha', 0.4); 
      drawnow
    end

    Cpre = dyn.pre(C, 0);
    if isEmptySet(Cpre)
      if verbose
        disp('returned empty')
      end
      Ct = Cpre;
      break;
    end

    Ct = intersect(Cpre, Safe);
    Ct = minHRep2(Ct);

    cc = Ct.chebyCenter;
    time=toc;

    iter = iter+1;
    
    if verbose
      disp(sprintf('iteration %d, %d ineqs, ball %f, time %f', ...
                 iter, size(Ct.A,1), cc.r, time));
    end
    if iter == maxIter
        break;
    end
  end

  if verbose && ~isEmptySet(Ct)
    disp(sprintf('finished with nonempty set after %d iterations!', iter))
  end
