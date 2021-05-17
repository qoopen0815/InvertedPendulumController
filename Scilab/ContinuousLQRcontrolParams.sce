sys_ss = syslin('c',A,B,C);    //The plant (continuous-time)

Q = diag([1, 1, 10, 10]);
R = 1.0E3;     //Usual notations x'Qx + u'Ru

Big=blockdiag(Q,R);    //Now we calculate C1 and D12
[w,wp]=fullrf(Big);C1=wp(:,1:4);D12=wp(:,5:$);   //[C1,D12]'*[C1,D12]=Big
P=syslin('c',A,B,C1,D12);    //The plant (continuous-time)

[K,X]=lqr(P)
