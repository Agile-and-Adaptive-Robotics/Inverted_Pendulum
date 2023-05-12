%BRUTE proportional motor only controller design

clear;
clc;
close all;

s = tf('s');

zeta = 0.6% Damping ratio
frequency_n = 2.5 % Natural Frequency Hz
omegan = 2*pi*frequency_n;
Ktot = 7 % Note the gain is calculated from the low frequency amplitude

% omegan_new = (1-0.4167*zeta+zeta^2)/0.01
% S1 = -zeta*omegan_new + omegan_new*((1 - zeta^2)^(1/2))*i
% S2 = -zeta*omegan_new - omegan_new*((1 - zeta^2)^(1/2))*i

num = Ktot*omegan^2;
den = s*(s^2 + 2*zeta*omegan*s + omegan^2);
Gp = num/den
H = 1;
bode(Gp);
title('Uncompensated Bode');
CLTF = feedback(Gp, H);
figure;
step(Gp);
title('Uncompensated Step');

figure
rlocus(Gp);

%P controller 1
Kp = 0.8;

Gc1 = Kp;
CLTF1 = feedback(Gc1*Gp,H);
figure;
bode(Gp);
hold on;
bode(CLTF1);
title('P1 Bode');
legend('Uncompensated', 'Compensated')

figure;
CLTF1 = feedback(Gc1*Gp,H);
step(Gp);
hold on;
step(CLTF1)
title('P1 Step Response');
legend('Uncompensated', 'Compensated')

%PI controller 1 // Tinker designed on the platform
Kp = 3;
Ki = 0.2;
Gc1 = Kp + Ki/s;
CLTF1 = feedback(Gc1*Gp,H);
figure;
bode(Gp);
hold on;
bode(CLTF1);
title('PI 1 Bode');
legend('Uncompensated', 'Compensated')

figure;
CLTF1 = feedback(Gc1*Gp,H);
step(Gp);
hold on;
step(CLTF1)
title('PI 1 Step Response');
legend('Uncompensated', 'Compensated')

%Lead controller
a = 4.5989;
T = 0.1097;
Gc2 = (1+a*T*s)/(1+T*s);
CLTF2 = feedback(Gc1*Gp,H);
figure;
bode(Gp);
hold on;
bode(CLTF2);
title('Lead Bode');
legend('Uncompensated', 'Compensated')

figure;
CLTF2 = feedback(Gc2*Gp,H);
step(Gp);
hold on;
step(CLTF2)
title('Lead Step Response');
legend('Uncompensated', 'Compensated')

%PI controller 3
Kp = 0.601;
Ki = 0.258;
Gc3 = Kp + Ki/s;
CLTF2 = feedback(Gc1*Gp,H);
figure;
bode(Gp);
hold on;
bode(CLTF2);
title('Gc 3 Bode');
legend('Uncompensated', 'Compensated')

figure;
CLTF2 = feedback(Gc3*Gp,H);
step(Gp);
hold on;
step(CLTF2)
title('Gc3 Step Response');
legend('Uncompensated', 'Compensated')