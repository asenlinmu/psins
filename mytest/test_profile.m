profile on
rv = randn(3,1)*0.1;  rv1 = rv+1;
m = rv2m(rv);
for k=1:100000
%     r1 = m2rv(m);
%     r2 = m2rv2(m);
    c1 = cros(rv,rv1);
    c2 = cross(rv,rv1);
end
profile viewer
% [rv, r1-rv, r2-rv]
