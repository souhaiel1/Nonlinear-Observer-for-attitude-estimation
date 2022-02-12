function w=Quaternion_model(t,v)
global a b c

q0=v(1);
q1=v(2);
q2=v(3);
q3=v(4);

% Quaternion model and the differential equations that describes the
% vari-ation of quaternion:


qvect = [q1 q2 q3];
skew_mat = [0 -q3 q2;
        q3 0 -q1;
        -q2 q1 0];
tt = eye(3)*q0 + skew_mat;
prod = 0.5*[-qvect;
            tt];
w = prod*([a b c]');
 
