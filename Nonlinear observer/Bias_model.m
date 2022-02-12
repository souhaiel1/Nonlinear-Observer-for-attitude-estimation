
function y=Bias_model(t,v)
    v1=v(1);
    v2=v(2);
    v3=v(3);

    % Bias model and differential equation that describes the variation of gyro bias
    
    std = 0.01;
    d1 = randn(1,1)*std^2;
    d2 = randn(1,1)*std^2;
    d3 = randn(1,1)*std^2;
    tau = 80;
    dv1dt = -v1/tau + d1;
    dv2dt = -v2/tau + d2;
    dv3dt = -v3/tau + d3;

y=[dv1dt;dv2dt;dv3dt];


