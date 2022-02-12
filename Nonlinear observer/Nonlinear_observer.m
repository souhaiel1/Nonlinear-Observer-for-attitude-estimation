function y= Nonlinear_observer(t,i)
global  K1 K2 d e f q1er q2er q3er q0er a b c


q0e=i(1);
q1e=i(2);
q2e=i(3);
q3e=i(4);
vxe=i(5);
vye=i(6);
vze=i(7);

% Nonlinear observer and the differential equations that describes it
% from eq 12  in the paper
N = eye(3)*80;
q_e = [-q1e -q2e -q3e; q0e -q3e q2e;q3e q0e -q1e;-q2e q1e q0e];
wg = [d; e; f];
bg = [vxe; vye; vze];
q_vect_er = [q1er(end); q2er(end); q3er(end)];

prod2 = wg-bg + K1*q_vect_er;

L1 = 0.5*q_e*prod2;
L2 = -inv(N)*bg-K2*q_vect_er;

dq0edt = L1(1);
dq1edt = L1(2);
dq2edt = L1(3);
dq3edt = L1(4);

dvxedt = L2(1);
dvyedt = L2(2);
dvzedt = L2(3);
%output
y=[dq0edt;dq1edt;dq2edt;dq3edt;dvxedt;dvyedt;dvzedt];

