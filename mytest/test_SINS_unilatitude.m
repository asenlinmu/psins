% Long-time SINS pure inertial navigation simulation on static base.
% See also  test_SINS_trj, test_SINS, test_SINS_GPS.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/06/2011
glvs
T = 60*60;  % total simulation time length
[nn, ts, nts] = nnts(4, 1);
avp0 = avpset([0;0;0], [100;0;0], [34;108;380]);
imuerr = imuseterr([0.01;0.001;50;5]*0);
[imu, avp1] = trjunilat(avp0, ts, T, imuerr);   % imu simulation
davp0 = avpseterr([-20;20;3], [0.01;0.01;0.01], [10;10;10]);
avp00 = avpadderr(avp0, davp0*0);
avp2 = inspure(imu, avp00);  % pure inertial navigation
avperr = avpcmp(avp2, avp1);
% insplot(avp1);
inserrplot(avperr);

