function V = expand(varargin)
% Find controlled invariant set (CIS) V \subseteq S
% V is CIS to be expanded 
%
% Usage:
%   - P = expand(pwd0, Safe, V, rhoPre)
%   - P = expand(pwd0, Safe, V, rhoPre, 'plot_stuff')
%   - P = expand(pwd0, Safe, V, rhoPre, 'debug')
%   - P = expand(pwd0, Safe, V, rhoPre, 'max_iter', max_iter)
%   - P = expand(pwd0, Safe, V, rhoPre, 'plot_stuff','debug')
%
% Inputs:
%   S - The safe set of the system
%       Assumed to be a union of polyhedra.
%   V - Initial invariant set to be expanded
%
%   plot_stuff - string
%                indicates that the system will plot figures of intermediate sets while operating.

%% Manage Inputs

pwd0 = varargin{1};
Safe = varargin{2};
V0 = varargin{3};
rhoPre = varargin{4};

%Assign defaults to the extra variables.
plot_stuff = false;
debug_flag = false;
max_iter = Inf;

arg_ind = 5;
while arg_ind <= nargin
    switch varargin{arg_ind}
    case 'plot_stuff'
        plot_stuff = true;
        arg_ind = arg_ind + 1;
    case 'debug'
        debug_flag = true;
        arg_ind = arg_ind + 1;
    case 'max_iter'
        if (nargin - arg_ind < 1) || (~isa(varargin{arg_ind+1},'double'))
            error('Expect a numerical argument after the ''max_iter'' string')
        end
        max_iter = varargin{arg_ind+1};
        arg_ind = arg_ind + 2; 
    otherwise
        error(['Unrecognized string input: ' varargin{arg_ind} ])
    end
end

%% Main Function
iter_num = 1;
vol = volumePolyUnion(V0);
V = V0.copy;
%Input V is assumed to be a PolyUnion.
if isa(V,'Polyhedron')
    V = PolyUnion(V);
end

if plot_stuff
    fig = figure;
    fig2 = figure;
end

% profile on;
counter = 0;
pre_V = V.copy;

while(1)
    counter = counter + 1;
    
    if debug_flag
        disp("Now doing iteration # "+num2str(iter_num));
    end
    
    pre_V = pre(pwd0, pre_V, rhoPre);
%     V_old = V;
    pre_V = IntersectPolyUnion(Safe,pre_V);
    V.add(pre_V.Set);    
%     V_saved = V;
    try
%        pre_V.merge();
        pre_V.reduce();

        1;
    catch
        %warning("merge failure.");
        %V = V_saved;
%         pre_V.reduce();
%         1;
    end
    V;
%     if(mod(counter,10)==0)
%       difference = setMinus3(V,V_old);      
% %       vol = volumePolyUnion(difference.convexHull);
%     end
    
%     fig = figure;
    if plot_stuff
        visual2(V,fig);
        visual(V, fig2);
    end
        
    
    % Termination Condition
    if(vol == 0 || iter_num >= max_iter)
        break;
    end
    
    iter_num = iter_num + 1;

end
