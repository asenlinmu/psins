
att = randn(3,1);  att=q2att(a2qua(att)); att(1:2)=0;
Cnb = a2mat(att);
phi = [1;2;3]*0.001;
Cnp = eye(3)+askew(phi);
Cpb = Cnp'*Cnb;  Cpb1 = q2mat(qaddphi(a2qua(att),phi));
att1 = m2att(Cpb); % aa2phi(att1,att)
datt = att1-att;
att(3)=-att(3); s = sin(att); c = cos(att);
C = [ -c(3), -c(1)*s(3), 0; 
    s(3), -c(1)*c(3), 0;
    0, -s(1), 1 ];
phi1 = C*datt; % Zhou Jiang-bin
[phi, -Cnb*a2Cwa(att)*datt, phi1, att/glv.deg]
