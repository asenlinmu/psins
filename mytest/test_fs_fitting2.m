%% ellipsoid
a = 1.5; b = 1; a2 = a*a; b2 = b*b;
afa = (0:0.01:1)'*2*pi;
r = sqrt(a2*b2./(a2*sin(afa).^2+b2*cos(afa).^2));
%% squqre
% a = 1;
% afa1 = -pi/4:0.1:pi/4; afa2 = afa1+2*pi/4; afa3 = afa2+2*pi/4; afa4 = afa3+2*pi/4;
% r1 = a./cos(afa1);
% afa = [afa1, afa2, afa3, afa4, 2*pi+afa1(1)]'; r = [r1,r1,r1,r1,r1(1)]';
% figure, plot(afa, r); grid on
for N=1:20
    [a0, anbn, w1] = fsfitting([afa,r], N);
    xx = a0; tt = afa;
    for n=1:N
        wt = n*w1*tt;
        xx = xx + anbn(n,1)*cos(wt) + anbn(n,2)*sin(wt);
    end
    % plot(afa,r, tt,xx, 'r'); grid on
    plot(r.*cos(afa),r.*sin(afa), xx.*cos(afa),xx.*sin(afa), 'r');  grid on
    L = 1.5*max(a,b); xlim([-L,L]); ylim([-L,L]);
end