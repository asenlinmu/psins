function [a, b, err] = pib0ls(y, L, ts)
global glv
    [len,m] = size(y);
    wt = (1:len)'*(glv.wie*ts);
    A = [ones(len,1), wt, wt.^2/2, -cos(wt), -sin(wt)];
    b = lscov(A, y);
    g = norm(y(end,:))/((len*ts)^2/2);
    a = b(3:5,:)/(g*cos(L)/glv.wie^2);
    %%
    err = y;
    for k=1:m
        err(:,k) = y(:,k)-A*b(:,k);
    end
    figure, plot(err), grid on
