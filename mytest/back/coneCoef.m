function [kk, phim] = coneCoef(hh, wm)
    n = length(hh);
    if n==2
        h1 = hh(1); h2 = hh(2); h = h1+h2;
        kk = 1/6*h^2/(h1*h2);
        if nargin==2
            phim = wm(1,:)+wm(2,:) + kk*cross(wm(1,:),wm(2,:));
            phim = phim';
        end
    else
        h1 = hh(1); h2 = hh(2); h3 = hh(3); h = h1+h2+h3;
        L = [h^3/6; h^5/5; 0];
        A = [h1*h2*(h1+h2), h1*h3*(h1+2*h2+h3), h2*h3*(h2+h3)];
        A(2,1) = A(1,1)*((h1+h2)^2+h1^2+h2^2);
        A(2,2) = A(1,2)*((h1+2*h2+h3)^2+h1^2+h3^2);
        A(2,3) = A(1,3)*((h2+h3)^2+h2^2+h3^2);
        A(3,:) = [1, 0, -1];
        kk = A^-1*L;    
        if nargin==2
            phim = wm(1,:)+wm(2,:)+wm(3,:) + ...
                    kk(1)*cross(wm(1,:),wm(2,:)) + ...
                    kk(2)*cross(wm(1,:),wm(3,:)) + ...
                    kk(3)*cross(wm(2,:),wm(3,:));
            phim = phim';
        end
    end

