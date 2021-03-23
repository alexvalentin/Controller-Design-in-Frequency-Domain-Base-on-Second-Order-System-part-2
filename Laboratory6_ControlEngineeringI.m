%% Laboratory 6 - Control Engineering I
%% CONTROLLER DESIGN IN FREQUENCY DOMAIN BASED ON SECOND ORDER SYSTEM - part 2-
%% GOALS

% To follow and to understand the design method steps 

% To check the resulted performance indicators

%% Problem
% For the process described by Hf(s) = 3.5 / s(0.5*s + 1) and the
% performance indicators [..] following the above described steps, design a
% Pd and a PID controller. Simulate the step and ramp output of the closed
% loop to highlight the performance indices.
%% Exercise (a) - PD Control

Hf = tf(3.5, [0.5 1 0])
bodemag(Hf), hold on

wf = 1/0.5; %1/Tf
sigma = 0.1
zeta = abs(log(sigma)) / sqrt (pi^2 + log(sigma)^2)

A = 1/4/sqrt(2)/zeta^2;

% A in decibels
AdB = 20 *log10(A);

A = tf(A, 1);
bodemag(A)

FN = -AdB + 1.84; %1.84 from Hf(blue) at freq 2
Vr = 10^(-FN/20);

Hd = Hf*Vr;
bode(Hd)

wt = 1.22 %%%%% frequency in 0 dB from Hd
wn = 2 * zeta * wt;

ts = 4/zeta/wn

cvdB = 2.15; %%%%%% read Hd(s) from bode(Hd) for w(freq)=1
cv = 10^(cvdB/20)  % >=2

deltawv = wt;
wt1 = wt;

ts_star = 3
wt2 = wt1 *ts/ts_star;
tau_d = 0.5;
Tn = tau_d * ts_star / ts;

Vr_PD = wt2 / wt1

Hr_PD = Vr_PD * tf([tau_d 1], [Tn 1])
Hd2 = minreal(Hf * Vr * Hr_PD)
bodemag(Hd2)

cv_PD_dB = 6.58; %%%%%% read Hd(s) from bode(Hd) for w(freq)=1
cv_PD = 10^(cv_PD_dB / 20)

deltawb_PD = wt2;

H0 = minreal (feedback(Vr * Hr_PD * Hf, 1))

figure, step(H0)
t = 0 : 0.1 : 80;
figure, lsim(H0, t, t)
 
%% Exercise (b) - PI Controller

Hf = tf(3.5, [0.5 1 0])
bodemag(Hf), hold on

wf = 1/0.5;
sigma = 0.05
zeta = abs(log(sigma)) / sqrt (pi^2 + log(sigma)^2)

A = 1/4/sqrt(2)/zeta^2;

% A in decibels
AdB = 20 *log10(A)

A = tf(A, 1);
bodemag(A)

FN = -AdB + 1.84 %1.84 from Hf(blue) at freq 2
Vr = 10^(-FN/20)

Hd = Hf*Vr;
bode(Hd)

wt = 0.95 %%%%% frequency in 0 dB from Hd
wn = 2 * zeta * wt;

ts = 4/zeta/wn

% cv =lim(s*Hd(s), s->0)
%%%%%% read Hd(s) from bode(Hd) for w(freq)=1
cv = 1.0512  %%% cv = lim(s*Hd), s->0 from Hd

deltawv = wt;
wt1 = wt;

ts_star = 0.8;
wt2 = wt1 *ts/ts_star;

tau_d = 0.5;
Tn = tau_d * ts_star / ts;

Vr_PD = wt2 / wt1

Hr_PD = Vr_PD * tf([tau_d 1], [Tn 1])
Hd2 = minreal(Hf * Vr * Hr_PD)
bodemag(Hd2)

cv_PD_dB = 15.2; %%%%% read from bodemag(Hd2) , Hd2 in w = 1
cv_PD = 10^(cv_PD_dB / 20)

deltawb_PD = wt2 % <= 15 rad/sec

% PI Controller Design
cvstar = 7;
wz = 0.1 * wt2;
wp = cv_PD /cvstar*wz;
Tz = 1/wz;
Tp = 1/wp;
VrPI = cvstar/ cv_PD

HPI = VrPI * tf([Tz 1], [Tp 1]); % PI controller tf
HdC = minreal (Vr  * Hr_PD * HPI * Hf) % open loop tf with all 3 controllers

bodemag(HdC)  % read when wt =1

cv_PID_db = 16.8 
cv_PD = 10 ^(cv_PID_db/20) 
H0 = feedback (HdC, 1)
 
figure, step(H0)
t = 0 : 0.1 : 80;
figure, lsim(H0, t, t)
