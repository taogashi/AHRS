clear all;
close all;

data=textread('LOG140709_1538.TXT');
x=[-0.0400; -0.1941; -0.0106; 0; 0; 0; -0.39; -0.23; -9.06; 51; 135; 318];
P=100*eye(12);

Q=diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.02, 0.02, 0.02, 0.01, 0.01, 0.01].^2);
R=diag([0.1, 0.1, 0.1, 1, 1, 1, 20, 20, 20].^2);
q_gyr = 0.01^2;
q_dgyr = 0.01^2;
q_acc = 0.02^2;
q_mag = 1^2;
r_gyr = 0.2^2;
r_acc = 2^2;
r_mag = 20^2;

gyr_raw = data(2:end,13:15)/180*pi; % rad/s
acc_raw = data(2:end,16:18);    % m_s^2
mag_raw = data(2:end,19:21);    % LSB
dt = (data(2:end, 22)-data(1:end-1, 22))/1000; % sec
time = zeros(length(dt)+1,1);

angle = zeros(3,length(dt));

for n=1:length(dt)
    [x,P,DCM,angle(:,n),debug] = AttitudeEKF(x,false,[1,1,1],dt(n),[gyr_raw(n,:)';acc_raw(n,:)';mag_raw(n,:)'],q_gyr,q_dgyr,q_acc,q_mag,r_gyr,r_acc,r_mag,eye(3));
    time(n+1)=time(n)+dt(n);
end

time = time(2:end);

subplot(2,1,1);
plot(time, angle(1,:)*57.6);
hold on;
plot(time, angle(2,:)*57.6,'r');
plot(time,angle(3,:)*57.6,'g');
grid on;
legend('roll','pitch','yaw');
%%
subplot(2,1,2);
ref_eular = data(2:end,10:12);
plot(time, ref_eular(:,1));
hold on;
plot(time, ref_eular(:,2),'r');
plot(time, ref_eular(:,3)-360,'g');
grid on;