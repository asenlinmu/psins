function dy = testfun(x, y)
    dy = y;
    dy(1) = sin(x);
    dy(2) = y(2);