% syms a w h t
a=randn; w=randn; t=randn; h=randn; t=randn;
E=-2*w*h*sin(a/2)^2; F=2*sin(a)*sin(w*h/2);
wm1 = [  
    -F*sin(w*(t+h/2))
    F*cos(w*(t+h/2))
    E ];
wm2 = [  
    -F*sin(w*(t+h+h/2))
    F*cos(w*(t+h+h/2))
    E ];
cross(wm1, wm2);
[    2*E*F*sin(w*(t+h))*sin(w*h/2); 
    -2*E*F*cos(w*(t+h))*sin(w*h/2); 
     F^2*sin(w*h)] ;
cross(cross(wm1, wm2), wm1)
cross(cross(wm1, wm2), wm2)
-2*E*F^2*sin(w*h/2)^2 
return

[cross(cross(wm1, wm2), wm1) -cross(cross(wm2, wm1), wm1)]
-askew(wm1)^2*wm2

 2*E*F^2*sin(w*h/2)*(sin(w*(t+h))*cos(w*(t+3*h/2))-cos(w*(t+h))*sin(w*(t+3*h/2)))
(sin(w*(t+h))*cos(w*(t+3*h/2))-cos(w*(t+h))*sin(w*(t+3*h/2)))
sin((w*h)-w*3*h/2)
sin(w*h/2) 
% [cos(t+h/2)-cos(t+3*h/2), 2*sin(t+h)*sin(h/2)]
% [sin(t+h/2)-sin(t+3*h/2), -2*cos(t+h)*sin(h/2)]

