clear;
close all;
A = csvread('Motor Only Bode Data.csv');
freq = A(:,1);
Amp = A(:,2);
Phase = A(:,3).*freq(:,1)*(-360);
% The three model parameters that mus be determined are the Gain, Zeta, and omegan
zeta = 0.6% Damping ratio
frequency_n = 2.5 % Natural Frequency Hz
omegan = 2*pi*frequency_n;
Ktot = 7 % Note the gain is calculated from the low frequency amplitude
num = Ktot*omegan^2
den = [1 2*zeta*omegan omegan^2 Ktot*omegan^2]
sys = tf(num,den);
w = logspace(-1,3);
[Mag_t,Phase_t] = bode(sys,w);
Mag_t = squeeze(Mag_t);
Phase_t = squeeze(Phase_t);
% % Motor model
% L = % Inductance (H)
% Kt = % Torque constant (N-m/A)
% R = % Resistance (Ohm)
% Kbac = % Back emf constant (V/(rad/s))
% I = % Rotor inertia (kg-m2)
% b = % Viscous Damping (N-m-s)
% Ktac = % Tachometer constant (V/(rad/s))
% Kamp = % Amplifier gain
% num_motor = Kt * Kamp * Ktac;
% den_motor = [I*L (b*L + I*R) (R*b + Kt*Kbac)];
% sys_motor = tf(num_motor,den_motor)
% [Mag_motor,Phase_motor] = bode(sys_motor,w);
% Mag_motor = squeeze(Mag_motor);
% Phase_motor = squeeze(Phase_motor);
% omegan_motor = sqrt((R*b + Kt*Kbac)/ (I*L))
% Zeta_motor = (b*L + I*R)/(2*omegan_motor*I*L)
% frequency_motor = omegan_motor/(2*pi)
% mech_time_constant = (R*I)/(Kbac*Kt)
% elec_time_constant = L/R
% roots_den_motor = roots(den_motor)
%Seperate Plots
figure
semilogx(freq,20*log10(Amp),'.-',w/(2*pi),20*log10(Mag_t))
grid
xlabel('Frequency (hz)')
ylabel('Amplitude (db)')
legend('Experimental Data','Theoritical Data')
figure
semilogx(freq,Phase,'.-',w/(2*pi),Phase_t)
grid
xlabel('Frequency (hz)')
ylabel('Phase (degrees)')
legend('Experimental Data','Theoritical Data','Motor Model')

%Subplots
figure
subplot(2,1,1), semilogx(freq,20*log10(Amp),'.-',w/(2*pi),20*log10(Mag_t))
grid
xlabel('Frequency (hz)')
ylabel('Amplitude (db)')
legend('Experimental Data','Theoritical Data')
subplot(2,1,2), semilogx(freq,Phase,'.-',w/(2*pi),Phase_t)
grid
xlabel('Frequency (hz)')
ylabel('Phase (degrees)')
legend('Experimental Data','Theoritical Data')