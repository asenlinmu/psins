clear all
glvs
[nn, ts, nts] = nnts(2, .1);
att = [0; 0; 0];
pos = posset(34, 108, 0);
vE = 500; vn = [vE; 0; 0];
eth = earth(pos,vn);
wbib = eth.wnin; fb = -eth.gcc;
wvm = [wbib; fb]'*ts; wvm = repmat(wvm, nn, 1);
ins = insinit([att;vn;pos], ts);
len = fix(3600/ts); res = zeros(len/nn,10);
ki = timebar(nn, len);
for k=1:nn:len
    ins = insupdate(ins, wvm);  ins.vn(3) = 0;
    res(ki,:) = [ins.avp; k*nts]';
    ki = timebar;
end
la = (1:length(res))*nts*vE/ins.eth.clRNh;
res(:,4) = res(:,4)-vE;
res(:,8) = res(:,8)-la';
insplot(res);

