clc
load('FlightData.mat');
fx=Acc_x.signals.values;
fy=Acc_y.signals.values;
fz=Acc_z.signals.values;
gx=Gyro_x.signals.values;
gy=Gyro_y.signals.values;
gz=Gyro_z.signals.values;

fx_time=[Acc_x.time fx];
fy_time=[Acc_y.time fy];
fz_time=[Acc_z.time fz];
gy_time=[Gyro_y.time gy];
gx_time=[Gyro_x.time gx];
gamma=0.99;

t_hat_acc=atan2(-fx,sqrt(fy.^2+fz.^2));
i=0:0.01:90.01;

[t_hat,phi_hat]=complementary_filter(gamma,fx,fy,fz,gx,gy,gz);
t_hat_time=[i' t_hat];
gy_time=[i' [0;gy]];
t_hat_acc_time=[i' [0;t_hat_acc]];
