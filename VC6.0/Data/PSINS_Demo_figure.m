% Figure for C++ processing results
% Make sure Matlab/PSINS Toolbox have been initialized!
glvs
PSINSDemo = 10;
switch PSINSDemo
    case 1, %% Demo_CIIRV3
        res = binfile('res.bin', 6);
        figure,
        subplot(131), plot(res(:,1:3:6)); grid on;
        subplot(132), plot(res(:,2:3:6)); grid on;
        subplot(133), plot(res(:,3:3:6)); grid on;
    case 2, %% Demo_CMaxMin
        res = binfile('res.bin', 5);
        figure,
        plot(res); grid on; legend('x', 'maxRes', 'minRes', 'maxpreRes', 'minpreRes');
    case 3, %% Demo_CVAR
        res = binfile('res.bin', 3);
        figure,
        plot(res); grid on; legend('x', 'mean', 'var');
    case 4, %% Demo_CVARn
        res = binfile('res.bin', 3);
        figure,
        plot(res); grid on; legend('x', 'meanx', 'stdx');
    case 5, %% Demo_CRAvar
        res = binfile('res.bin', 2);
        figure,
        plot(res); grid on; legend('x', 'sqrt(RAvar)');
    case 6, %% Demo_CSINS_static
        avp = binfile('ins.bin', 16);
        insplot(avp(:,[1:9,end]));
    case 7, %% Demo_CAlignsv
        att = binfile('aln.bin', 4);
        T1 = find(diff(att(:,end))<0,1);
        insplot(att(T1+1:end,:),'a');
        subplot(211), plot(att(1:T1,end), att(1:T1,1:2)/glv.deg, 'r');
        subplot(212), plot(att(1:T1,end), att(1:T1,3)/glv.deg, 'r');
    case 8, %% Demo_CAlign_CSINS
        att = binfile('aln.bin', 4);
        insplot(att,'a');
        avp = binfile('ins.bin', 16);
        insplot(avp);
    case 10, %% Demo_SINS/GNSS
        ins = binfile('ins.bin', 16+3);
        insplot(ins(:,[1:16]));
        avpcmpplot(ins(:,[17:19,16]), ins(:,[1:3,16]), 'a', 'mu');
        xkpk = binfile('kf.bin', (15+6)*2+2);
        psinstypedef(156);
        kfplot(xkpk(:,[1:30,end]));
        stateplot(xkpk(:,end-1:end));
end

