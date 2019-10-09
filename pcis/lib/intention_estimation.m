function Iv = intention_estimation(q, a, con)
% Iv \in { 'bnd', 'ann', 'cau' }
Iv = 'bnd';
is_ann = false;
is_cau= false;


a_ann = min(max(con.K_ann*q, con.aL_min - con.dLmin), con.aL_max-con.dLmax);
a_cau = min(max(con.K_cau*q-con.K_cau(4)*con.vL_des, con.aL_min- con.dLmin), con.aL_max-con.dLmax);

if abs(a-a_ann) <= con.dLmax
    is_ann = true;
end
if abs(a-a_cau) <= con.dLmax
    is_cau = true;
end

if is_ann && ~is_cau
    Iv = 'ann';
elseif is_cau && ~is_ann
    Iv = 'cau';
end
1;
