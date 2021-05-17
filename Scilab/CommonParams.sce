clear;

// echobot v1 params
m_w = 0.09;
m_p = 0.646;
r_w = 0.04;
r_p = 0.04;
I_w = 0.000072;
I_p = 0.00084261;
I_m = 1.74636E-06;
n = 56.0;
// 実験値
// k_t = 0.006141979;
// datasheet
k_t = 8.4*0.01;
// 実験地
k_b = 0.011333707;
// datasheet
// k_b = 2.05;
R = 0.666666667;
V_off = 0.021923416;
g = 9.8;
v_max = 7;

a_11 = (m_w+m_p)*r_w*r_w + 2*m_p*r_w*r_p + m_p*r_p*r_p + I_p + I_w;
a_12 = (m_w+m_p)*r_w*r_w + m_p*r_w*r_p + I_w;
a_21 = (m_w+m_p)*r_w*r_w + m_p*r_w*r_p + I_w;
a_22 = (m_w+m_p)*r_w*r_w + I_w + n*n*I_m;

delta = a_11*a_22-a_12*a_21;

A = [0 1 0 0;
     a_22*m_p*g*r_p/delta 0 0 (a_12*n*n*k_t*k_b/R)/delta;
     0 0 0 1;
     -a_21*m_p*g*r_p/delta 0 0 (-a_11*n*n*k_t*k_b/R)/delta];
B = [0;
     (-a_12*n*k_t/R)/delta;
     0;
     (a_11*n*k_t/R)/delta];
C = diag([1 1 1 1]);

dt = 0.002;
X_init = [0.3 0 0 0]; // attitude 20 deg
