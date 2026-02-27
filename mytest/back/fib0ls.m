function [a, err] = fib0ls(y, L, ts)
global glv
    [len,m] = size(y);
    wt = (1:len)'*(glv.wie*ts);
    A = [ones(len,1), cos(wt), sin(wt)];
    b = lscov(A, y);
    g = norm(b(1,:)+b(2,:));  g1 = norm(b(1,:)+b(3,:));
    a = b/(g*cos(L));
    %%
    err = y;
    for k=1:m
        err(:,k) = y(:,k)-A*b(:,k);
    end
    figure, plot(err), grid on

