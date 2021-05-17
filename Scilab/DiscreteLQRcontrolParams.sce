sys_ss = syslin('c',A,B,C);    //The plant (continuous-time)
sys_d = dscr(sys_ss, dt);

Q = diag([100, 1, 10, 1]);
R = 1.0E1;     //Usual notations x'Qx + u'Ru

Big=blockdiag(Q,R);    //Now we calculate C1 and D12
[w,wp]=fullrf(Big);C1=wp(:,1:4);D12=wp(:,5:$);   //[C1,D12]'*[C1,D12]=Big
P=syslin('d',sys_d.a,sys_d.b,C1,D12);    //The plant (continuous-time)

[Kd,X]=lqr(sys_d, Q, R)

spec(sys_d.a+sys_d.b*Kd)
norm(sys_d.a'*X*sys_d.a-(sys_d.a'*X*sys_d.b)*pinv(sys_d.b'*X*sys_d.b+R)*(sys_d.b'*X*sys_d.a)+Q-X,1)
