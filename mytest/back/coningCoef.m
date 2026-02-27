function [coef, err] = coningCoef(hh, afa, omega)
    if length(hh)==1
        hh = max(hh,2); % must be hh>=2
        hh = ones(hh,1);
    end
    f = min(hh); hh = hh/f;  % scale
    n = length(hh); Tm = sum(hh);
    A = zeros(n,n-1);
    shn = sinserial(hh(end)/2,n);
    for i=1:n-1
        hin = 2*sum(hh(i:n))-hh(i)-hh(n);
        shi = sinserial(hh(i)/2,n);
        shin = sinserial(hin/2,n);
        A(:,i) = v2M(v2M(shi)*shn)*shin;
    end
    % -4*A*K = 1/2*dphiz
    phiz = sinserial(Tm, n+1);
    coef = -1/8*A(1:n-1,:)^-1*phiz(2:n);
    if nargout>1
        err = -afa^2*omega^(2*n+1)/(Tm*f) * (4*A(n,:)*coef+phiz(n+1)/2)*f^(2*n+1);
        err(2,1) = conedrift(afa, omega, Tm*f/n, n);
    end

function M = v2M(v)
    len = length(v);
    M = zeros(len);
    for k=1:len
        M(k:end,k) = v(1:len-k+1);
    end
        
function sc = sinserial(a, n)
    sc = zeros(n,1);
    for k=1:n
        sc(k) = (-1)^(k+1)*a^(2*k-1)/factorial(2*k-1);
    end
    
% %     M = v2M1([1;2;3]);
% %     sc = sinserial(1,15);
%     x=1; y=2; z=3;
%     f3 = factorial(3); f5 = factorial(5); f7 = factorial(7); f9=factorial(9);
%     a=(x^1-x^3/f3+x^5/f5)*(y^1-y^3/f3+y^5/f5)*(z^1-z^3/f3+z^5/f5-z^7/f7+z^9/f9);
%     b=v2M1(v2M1(sinserial(x,3))*sinserial(y,3))*sinserial(z,5);
%     [a,sum(b)]'
%     a = 1;
% 
% function M = v2M1(v)
%     len = length(v);
%     M = zeros(2*len-1,len);
%     for k=1:len
%         M(k:k+len-1,k) = v;
%     end

