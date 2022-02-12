% Implementation of the paper entitled : Posture and body acceleration tracking by inertial and magnetic sensing:
% Application in behavioral analysis of free-ranging animals

%---------------- Souhaiel Ben Salem --------------


clear all;close all;clc

global q1er q2er q3er q0er d e f a b c K1 K2

%%%%%%%%%%%%%%%%%%%%%%%%%%
%Conditions initiales%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
Tfinal=50;                                                    % Final integration time (s)
pech=0.01;                                                    % Sampling period
num_iter=8;                                                   % Number of iterations of the iterative algorithm
alpha=1/3;                                                    % Constant which fixes the speed of convergence of the iterative algorithm
temps=0;                                                      % Sampling time initialization

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initial conditions for the model%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q0=1;
q1=0;
q2=0;
q3=0;
Y0=[q0 q1 q2 q3];
V0=[-2 1 0.5];
V=V0;
Q=Y0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initial conditions for the observer%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q0e=0.3;q1e=0.5;q2e=0.8;q3e=0.7;
vxe=0;vye=0;vze=0;
Y00=[q0e q1e q2e q3e vxe vye vze];
Qest=[q0e q1e q2e q3e];
vest=[vxe vye vze];

%%%%%%%%%%%%%%%%%%%%%%%
an=[0;0;0;1];                                                % Gravity field in fixed frame
mn=[0;0.5;0;sqrt(3)/2];                                      % Terrestrial magnetic pole in the fixed frame

%Initialization of inertial measurement vectors
% Angular velocity
omega_x=[];
omega_y=[];
omega_z=[];
% Acceleration
ba_x=[];
ba_y=[];
ba_z=[];
% Magnetic field
bm_x=[];
bm_y=[];
bm_z=[];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Accelerations along the x axis%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ax1(1:500)=0;
ax1(501:600)=0.8;
ax1(601:700)=0;
ax1(701:800)=-0.9;
ax1(801:900)=0;
ax1(901:1000)=0.5;
ax1(1001:1600)=0;
ax1(1601:1700)=1;
ax1(1701:2500)=0;
ax1(2501:2600)=0.75;
ax1(2601:2700)=0;
ax1(2701:2800)=-0.88;
ax1(2801:2900)=0;
ax1(2901:3000)=-1.5;
ax1(3001:3600)=0;
ax1(3601:3700)=-0.9;
ax1(3701:4500)=0;
ax1(4501:4600)=0.65;
ax1(4601:4700)=0;
ax1(4701:4800)=-0.75;
ax1(4801:4900)=0;
ax1(4901:5000)=0.65;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Accelerations along the y axis%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:1000
    ay1(i)=0;
end
for i=1001:1100
    ay1(i)=1;
end
for i=1101:1200
    ay1(i)=0;
end
for i=1201:1300
    ay1(i)=-0.6;
end
for i=1301:1400
    ay1(i)=0;
end
for i=1401:1500
    ay1(i)=0.9;
end
for i=1501:2000
    ay1(i)=0;
end
for i=2001:2100
    ay1(i)=1.2;
end
for i=2101:2200
    ay1(i)=0;
end
for i=2201:2300
    ay1(i)=0.9;
end
for i=2301:2400
    ay1(i)=0;
end
for i=2401:2500
    ay1(i)=-1.2;
end
for i=2501:3000
    ay1(i)=0;
end
for i=3001:3100
    ay1(i)=0.6;
end
for i=3101:3200
    ay1(i)=0;
end
for i=3201:3300
    ay1(i)=2;
end
for i=3301:3400
    ay1(i)=0;
end
for i=3401:3500
    ay1(i)=-0.44;
end
for i=3501:4000
    ay1(i)=0;
end
for i=4001:4100
    ay1(i)=0.2;
end
for i=4101:4200
    ay1(i)=0;
end
for i=4201:4300
    ay1(i)=-0.95;
end
for i=4301:4400
    ay1(i)=0;
end
for i=4401:4500
    ay1(i)=1.6;
end
for i=4501:5000
    ay1(i)=0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Accelerations along the z axis%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:600
az1(i)=0;
end
for i=601:650
az1(i)=-0.8;
end
for i=651:700
az1(i)=0.8;
end
for i=701:1500
az1(i)=0;
end
for i=1501:1600
az1(i)=1.3;
end
for i=1601:1700
az1(i)=0;
end
for i=1701:1800
az1(i)=-0.8;
end
for i=1801:1900
az1(i)=0;
end
for i=1901:2000
az1(i)=0.9;
end
for i=2001:2600
az1(i)=0;
end
for i=2601:2700
az1(i)=0.87;
end
for i=2701:3500
az1(i)=0;
end
for i=3501:3600
az1(i)=0.3;
end
for i=3601:3700
az1(i)=0;
end
for i=3701:3800
az1(i)=-0.9;
end
for i=3801:3900
az1(i)=0;
end
for i=3901:4000
az1(i)=1.7;
end
for i=4001:4500
az1(i)=0;
end
for i=4501:4600
az1(i)=-1.3;
end
for i=4601:5000
az1(i)=0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%estimated accelerations%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
acx=0;
acy=0;
acz=0;
aest=[acx;acy;acz];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initial conditions of the recursive least squares algorithm%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
qps=[1;0;0;0];
Qps=[1 0 0 0];
Qinv=[Y00(1) -Y00(2) -Y00(3) -Y00(4)];                              % Inverse of the estimated Quaternion 
qe=Quaternion_product(Qinv',qps);                                   % Quaternion product
q0er=qe(1,1);q1er=qe(2,1);q2er=qe(3,1);q3er=qe(4,1);                % Solution

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Models and Nonlinear Observer%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% The equations related to theoretical variation of angular velocity (Theoretical angular velocity)
% following eq.  (14) :
wgx(1:5000)=0;
wgy(1:5000)=0;
wgz(1:5000)=0;

for t=1:2500
    wgx(t) = -0.8*sin(1.2*t*pech);
    wgy(t) = 1.1*cos(0.5*t*pech);
    wgz(t) = 0.4*sin(0.3*t*pech);
end

for t=2501:5000
    wgx(t) = 1.3*sin(1.4*t*pech);
    wgy(t) = -0.6*cos(-0.3*t*pech);
    wgz(t) = 0.3*sin(0.5*t*pech);
end
%%
%Nonlinear observer
for k1=1:Tfinal/pech

    a=wgx(k1);              % Theoretical angular velocity along x-axis
    b=wgy(k1);              % Theoretical angular velocity along y-axis
    c=wgz(k1);              % Theoretical angular velocity along z-axis
   
    TSPAN=[(k1-1)*pech k1*pech];
    [TOUT,YOUT]=ode45('Bias_model',TSPAN,V0);
    
    Temps(k1)=TOUT(end);
    v1(k1)=YOUT(end,1);
    v2(k1)=YOUT(end,2);
    v3(k1)=YOUT(end,3);
   
    V0=[v1(end) v2(end) v3(end)];
    V=[V;V0];
    
    % The equations related to measured angular velocity, following eq.(6):
    delta_b = 0.01;
    d= a +  v1(k1) +randn(1,1)*(delta_b^2);                                    % Angular velocity measured along x
    e= b +  v2(k1)+ randn(1,1)*(delta_b^2);                                    % Angular velocity measured along y
    f= c +  v3(k1)+ randn(1,1)*(delta_b^2);                                    % Angular velocity measured along z
    omega_x=[omega_x,d];
    omega_y=[omega_y,e];
    omega_z=[omega_z,f];
    
 %%   
    % Gains of nonlinear observer
    if k1<=300
        K1=100;
        K2=200;
    else
        K1=100;
        K2=200;
    end
    
    [TOUT,YOUT]=ode45('Nonlinear_observer',TSPAN,Y00);
    
    % obtaining estimated values of quaternion and bias:
    
    Temps(k1)=TOUT(end);
    q0e(k1)=YOUT(end,1);
    q1e(k1)=YOUT(end,2);
    q2e(k1)=YOUT(end,3);
    q3e(k1)=YOUT(end,4);
    vxe(k1)=YOUT(end,5);
    vye(k1)=YOUT(end,6);
    vze(k1)=YOUT(end,7);
   
    Y00=[q0e(end),q1e(end),q2e(end),q3e(end),vxe(end),vye(end),vze(end)];
    qest=[q0e(end),q1e(end),q2e(end),q3e(end)]/norm([q0e(end),q1e(end),q2e(end),q3e(end)]);
    qest0(k1)=qest(1);
    qest1(k1)=qest(2);
    qest2(k1)=qest(3);
    qest3(k1)=qest(4);
    Qest=[Qest;qest(1) qest(2) qest(3) qest(4)];
    vest=[vest;vxe(end) vye(end) vze(end)];

    [TOUT,YOUT]=ode45('Quaternion_model',TSPAN,Y0);
    
   %%
    % obtaining the estimated values of quaternion model:
    Temps(k1)=TOUT(end);
    q0(k1) = YOUT(end,1);
    q1(k1) = YOUT(end,2);
    q2(k1) = YOUT(end,3);
    q3(k1) = YOUT(end,4);
    
    qu=[q0(end) q1(end) q2(end) q3(end)];
    Q=[Q;qu];
    temps=[temps Temps(end)];
    Y0=qu;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Rotation Matrix C(q)%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    C=[2*(q0(end)^2+q1(end)^2)-1 2*(q1(end)*q2(end)+q0(end)*q3(end)) 2*(q1(end)*q3(end)-q0(end)*q2(end));2*(q1(end)*q2(end)-q0(end)*q3(end)) 2*(q0(end)^2+q2(end)^2)-1 2*(q0(end)*q1(end)+q2(end)*q3(end));2*(q0(end)*q2(end)+q1(end)*q3(end)) 2*(q2(end)*q3(end)-q0(end)*q1(end)) 2*(q0(end)^2+q3(end)^2)-1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% estimated accelerations %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
    M=[2*(qest(1)^2+qest(2)^2)-1 2*(qest(2)*qest(3)+qest(1)*qest(4)) 2*(qest(2)*qest(4)-qest(1)*qest(3));2*(qest(2)*qest(3)-qest(1)*qest(4)) 2*(qest(1)^2+qest(3)^2)-1 2*(qest(1)*qest(2)+qest(3)*qest(4));2*(qest(1)*qest(3)+qest(2)*qest(4)) 2*(qest(3)*qest(4)-qest(1)*qest(2)) 2*(qest(1)^2+qest(4)^2)-1]; 
    
    % Creating the measured acceleration from accelerometer, following eq.(6):
    delta_f = 0.002;
    noise_acc = [randn(1,1)*(delta_f^2); randn(1,1)*(delta_f^2); randn(1,1)*(delta_f^2)];
    ba = C*([ax1(k1); ay1(k1); az1(k1) + 1]) + noise_acc;
    
    ba_x=[ba_x,ba(1)];
    ba_y=[ba_y,ba(2)];
    ba_z=[ba_z,ba(3)];
    norme=(norm(ba))^2;
    if ((norme-1<=0.006)&&(norme-1>=-0.006))
    ba=ba;
    acx(k1)=ax1(k1);
    acy(k1)=ay1(k1);
    acz(k1)=az1(k1);
    else
    ae=inv(M)*(ba-(M*[an(2);an(3);an(4)]))-(randn(3,1)*0.002^2);
    acx(k1)=ae(1);
    acy(k1)=ae(2);
    acz(k1)=ae(3);
    ba=ba-(M*ae);
    end
    ba=[0;ba(1);ba(2);ba(3)];
    %%
%%%%%%%%%%%%%%%%%%%%%
%champ magnétique%%%%
%%%%%%%%%%%%%%%%%%%%%
    % Creating the measured magnetic field from magnetometer
    delta_h = 0.007;
    noise_h = [randn(1,1)*(delta_h^2); randn(1,1)*(delta_h^2); randn(1,1)*(delta_h^2)];
    bm = C*(mn(2:4)) + noise_h;
    
    bm_x=[bm_x,bm(1)];
    bm_y=[bm_y,bm(2)];
    bm_z=[bm_z,bm(3)];
    bm=[0;bm(1);bm(2);bm(3)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
%The Iterated least squares algorithm%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    qps=qest';
    % The Iterated least squares algorithm, following section 5.3.1 of the
    % paper
    for i=1:num_iter
        %%%%%%%%%%%%%%%%
        %estimated mn %%%%%%
        %%%%%%%%%%%%%%%%
        qps_inv = [qps(1); -qps(2); -qps(3); -qps(4)];
        mn_est = Quaternion_product(qps,(Quaternion_product(bm,qps_inv)));
        
        %%%%%%%%%%%%%%%%%
        %estimated an %%%%%%%
        %%%%%%%%%%%%%%%%%
        gvect_est = Quaternion_product(qps,(Quaternion_product(ba,qps_inv)));
        %%%%%%%%%%%%%%%%%%%%%%%%%
        %navigation error %%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%
        dan_est = an - gvect_est;
        dmn_est = mn - mn_est;
        z = [dmn_est(2:end); dan_est(2:end)];
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Observation Matrix H%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        mnx=[0 -mn(4) mn(3);mn(4) 0 -mn(2);-mn(3) mn(2) 0];
        anx=[0 -an(4) an(3);an(4) 0 -an(2);-an(3) an(2) 0];
        H=[-2*mnx;-2*anx];
        %%%%%%%%%%%%%%%%%%%%
        %Pseudo-inverse H*%%
        %%%%%%%%%%%%%%%%%%%%
        H_pseudo_inverse = (inv((H')*H))*(H');
        
        %%%%%%%%%%%%%%%%%%%%%%%
        %updating qe%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%
        q_e=alpha*H_pseudo_inverse*z;
        q_e=[1;q_e(1);q_e(2);q_e(3)];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Mupdating the estimated quaternion%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        qps = Quaternion_product(q_e, qps);
        
        qps=(1/norm(qps))*qps;
        
    end
    
Qps=[Qps;qps'];                                                           % Storage matrix of the measurement quaternion for the observer

Qinv=[qest(1) -qest(2) -qest(3) -qest(4)];                                % Inverse of estimated Quaternion 
qe=Quaternion_product(Qinv',qps);                                         % quaternion product (error epsilon)

q0er(k1)=qe(1,1);q1er(k1)=qe(2,1);q2er(k1)=qe(3,1);q3er(k1)=qe(4,1);      % Quaternion of the error
end

%%%%%%%%%%%%%%%%%%%
% Quaternion vrai%%
%%%%%%%%%%%%%%%%%%%
q0v=Q(:,1);
q1v=Q(:,2);
q2v=Q(:,3);
q3v=Q(:,4);
%%%%%%%%%%%%%%%%%%%%%%
% Quaternion mesuré%%%
%%%%%%%%%%%%%%%%%%%%%%
q0ps=Qps(:,1);
q1ps=Qps(:,2);
q2ps=Qps(:,3);
q3ps=Qps(:,4);
%%%%%%%%%%%%%%%%%%%%%%
% Quaternion estimé%%%
%%%%%%%%%%%%%%%%%%%%%%
q0est=Qest(:,1);
q1est=Qest(:,2);
q2est=Qest(:,3);
q3est=Qest(:,4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%Main_plots%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Inertial/magnetic data and linear acceleration
figure(1)
t = linspace(0,50,5000);

% Angular velocity
subplot(4,4,1)
plot(t,omega_x)
title('Angular velocity')

subplot(4,4,5)
plot(t,omega_y)

subplot(4,4,9)
plot(t,omega_z)

% Acceleration
t = linspace(0,50,5000);
subplot(4,4,2)
plot(t,ba_x,'r')
title('Acceleration')

subplot(4,4,6)
plot(t,ba_y,'r')

subplot(4,4,10)
plot(t,ba_z,'r')

% Magnetic field
subplot(4,4,3)
plot(t,bm_x,'k')
title('Magnetic field')

subplot(4,4,7)
plot(t,bm_y,'k')

subplot(4,4,11)
plot(t,bm_z,'k')

% Linear Acceleration
subplot(4,4,4)
plot(t,ax1,'g')
title('Linear Acceleration')

subplot(4,4,8)
plot(t,ay1,'g')

subplot(4,4,12)
plot(t,az1,'g')

%% Plotting the true and estimated quaternion and gyro bias
figure(2)

% Quaternion estimation
subplot(4,2,1)
t = linspace(0,50,5001);
plot(t,q0est,'--b', t, q0v,'--r')
title('Quaternion estimation')
legend('Non Linear observer', 'Theoretical model')

subplot(4,2,3)
plot(t,q1est,'--b', t, q1v,'--r')

subplot(4,2,5)
plot(t,q2est,'--b', t, q2v,'--r')

subplot(4,2,7)
plot(t,q3est,'--b', t, q3v,'--r')

% Bias Estimation
subplot(4,2,2)
plot(t,vest(:,1), '--b',t, V(:,1),'--r')
legend('Non Linear observer', 'Theoretical model')
title('Bias estimation')

subplot(4,2,4)
plot(t,vest(:,2), '--b',t, V(:,2),'--r')

subplot(4,2,6)
plot(t,vest(:,3), '--b',t, V(:,3),'--r')


% test
% figure() 
% plot(t,abs(q1v - q1est)); 
% title('error on quaternion')