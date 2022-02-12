
%Quatrernion product Function
function[quaternion]=Quaternion_product(quaternion1,quaternion2)
% Simplification of the writing of quaternions
p0=quaternion1(1,1);
p1=quaternion1(2,1);
p2=quaternion1(3,1);
p3=quaternion1(4,1);

q0=quaternion2(1,1);
q1=quaternion2(2,1);
q2=quaternion2(3,1);
q3=quaternion2(4,1);

% The equations that describes the quaternion product:
qvecta = [p1 p2 p3]';
qvecta_ss = [0 -p3 p2; p3 0 -p1; -p2 p1 0];
qb = [q0 q1 q2 q3];
prod = [p0 -qvecta';
        qvecta p0*eye(3)+qvecta_ss];

% Return the calculated quaternion
quaternion = prod*(qb.');
return


