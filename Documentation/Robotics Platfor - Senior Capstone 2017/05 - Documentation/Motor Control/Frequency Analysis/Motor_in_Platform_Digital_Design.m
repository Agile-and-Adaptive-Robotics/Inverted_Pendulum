%BRUTE proportional motor only controller design

clear;
clc;
close all;

s = tf('s');

zeta = 0.5% Damping ratio
frequency_n = 1.5 % Natural Frequency Hz
omegan = 2*pi*frequency_n;
Ktot = 2; 

% omegan_new = (1-0.4167*zeta+zeta^2)/0.01
% S1 = -zeta*omegan_new + omegan_new*((1 - zeta^2)^(1/2))*i
% S2 = -zeta*omegan_new - omegan_new*((1 - zeta^2)^(1/2))*i

%Open loop TF
num = Ktot*omegan^2;
den = s*(s^2 + 2*zeta*omegan*s + omegan^2);
Gp = num/den
H = 1;

Ts = 0.025;
sysD = c2d(Gp, Ts, 'ZOH')
sysW = d2c(sysD, 'tustin')

bode(sysW);
title('Uncompensated Bode');
CLTF = feedback(Gp, H);
figure;
step(sysW);
title('Uncompensated Step');

figure
rlocus(sysD);
zgrid
axis([-1 1 -1 1]);
axis('equal')

%Lead controller design
a = 2.7;
T = 0.3108;

GcW1 = (1+a*T*s)/(1+T*s);
GcD1 = c2d(GcW1, Ts, 'tustin')

figure;
bode(sysW);
hold on;

CLTFW = feedback(GcW1*sysW,H);
bode(CLTFW);
title('Lead 1 Bode');
legend('Uncompensated', 'Compensated')

figure;
CLTFD = feedback(GcD1*sysD,H);
step(sysD);
hold on;
step(CLTFD)
title('Lead 1 Step Response');
legend('Uncompensated', 'Compensated')

%Lead controller design 1
a = 0.0447
T = 0.473

GcW = (1+a*T*s)/(1+T*s);
GcD = c2d(GcW, Ts, 'tustin')

figure;
bode(sysW);
hold on;

CLTFW = feedback(GcW*sysW,H);
bode(CLTFW);
title('Lead 1 Bode');
legend('Uncompensated', 'Compensated')

figure;
CLTFD = feedback(GcD*sysD,H);
step(sysD);
hold on;
step(CLTFD)
title('Lead 1 Step Response');
legend('Uncompensated', 'Compensated')



% %Lag controller design 2
% a = 0.523
% T = 1.542
% 
% GcW = (1+a*T*s)/(1+T*s);
% GcD = c2d(GcW, Ts, 'tustin')
% 
% figure;
% bode(sysW);
% hold on;
% 
% CLTFW = feedback(GcW*sysW,H);
% bode(CLTFW);
% title('Lag 1 Bode');
% legend('Uncompensated', 'Compensated')
% 
% figure;
% CLTFD = feedback(GcD*sysD,H);
% step(sysD);
% hold on;
% step(CLTFD)
% title('Lag 1 Step Response');
% legend('Uncompensated', 'Compensated')


% %Lead Lag controller design
% a = 0.501
% T = 1.345
% 
% GcW2 = (1+a*T*s)/(1+T*s);
% GcD2 = c2d(GcW2, Ts, 'tustin')
% 
% figure;
% bode(sysW);
% hold on;
% 
% CLTFW = feedback(GcW1*GcW2*sysW,H);
% bode(CLTFW);
% title('Lead Lag Bode');
% legend('Uncompensated', 'Compensated')
% 
% figure;
% CLTFD = feedback(GcD1*sysD,H);
% step(sysD);
% hold on;
% step(CLTFD)
% title('Lead Lag Step Response');
% legend('Uncompensated', 'Compensated')
% 
% %PI controller 1
% Kp = 3;
% Ki = 0.2;
% Gc1 = Kp + Ki/s;
% CLTF1 = feedback(Gc1*Gp,H);
% figure;
% bode(Gp);
% hold on;
% bode(CLTF1);
% title('Gc1 Bode');
% legend('Uncompensated', 'Compensated')
% 
% figure;
% CLTF1 = feedback(Gc1*Gp,H);
% step(Gp);
% hold on;
% step(CLTF1)
% title('Gc1 Step Response');
% legend('Uncompensated', 'Compensated')
% 
% %PI controller 2
% Kp = 0.493;
% Ki = 0.162;
% Gc2 = Kp + Ki/s;
% CLTF2 = feedback(Gc1*Gp,H);
% figure;
% bode(Gp);
% hold on;
% bode(CLTF2);
% title('Gc 2 Bode');
% legend('Uncompensated', 'Compensated')
% 
% figure;
% CLTF2 = feedback(Gc2*Gp,H);
% step(Gp);
% hold on;
% step(CLTF2)
% title('Gc2 Step Response');
% legend('Uncompensated', 'Compensated')
% 
% %PI controller 3
% Kp = 0.243;
% Ki = 0.043;
% Gc3 = Kp + Ki/s;
% CLTF2 = feedback(Gc1*Gp,H);
% figure;
% bode(Gp);
% hold on;
% bode(CLTF2);
% title('Gc 3 Bode');
% legend('Uncompensated', 'Compensated')
% 
% figure;
% CLTF2 = feedback(Gc3*Gp,H);
% step(Gp);
% hold on;
% step(CLTF2)
% title('Gc3 Step Response');
% legend('Uncompensated', 'Compensated')