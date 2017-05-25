function pt_out = triangulate4(pt1, pt2,origin1,origin2,R1,R2)
%http://www.iim.cs.tut.ac.jp/~kanatani/papers/sstriang.pdf (iterative)
if nargin == 0
    untitled2
else
    fx = 529.1;
    fy = 529.1;
    cx = 350.6;
    cy = 182.2;

    K = [fx 0 cx; 0 fy cy; 0 0 1];
    P1 = K*R1'*[eye(3), -origin1];
    P2 = K*R2'*[eye(3), -origin2];
    F = getFundamentalMatrix2(P1,P2);
    pt1_h = [pt1;1];
    pt2_h = [pt2;1];
    
    u = reshape(F',9,1);
    P = diag([1,1,0]);
    
    f = 1;
    x_origin  = pt1_h;
    x1_origin = pt2_h;
    
    dx = [0;0;0]; dx1 = [0;0;0];
    k = 0;
    E  = 10
    E0 = 20
    while (abs(E-E0)>0.00001)
        k = k+1
        E0 = E;
        x_hat  = x_origin  - dx;
        x1_hat = x1_origin - dx1;
        epsil = get_eps(x_hat,x1_hat,dx,dx1,f);
        V = get_V(x_hat,x1_hat,f);
        den = u'*V*u;
        mult = u'*epsil*P /den;
        dx  = mult*F*x1_hat;
        dx1 = mult*F'*x_hat;
        E = norm(dx)^2 + norm(dx1)^2
    end   
    
    pt_out = triangulate(x_hat(1:2), x1_hat(1:2),origin1,origin2,R1,R2);
    
end

end

function V = get_V(x_hat,x_hat1,f)
    x  = x_hat(1);  y  = x_hat(2);
    x1 = x_hat1(1); y1 = x_hat1(2);
    x2 = x*x; x12 = x1*x1;
    y2 = y*y; y12 = y1*y1;
    V=[x2+x12,x1*y1, f*x1,  x*y,   0,     0,   f*x,0,  0;
       x1*y1, x2+y12,f*y1,  0,     x*y,   0,   0,  f*x,0;
       f*x1,  f*y1,  f*f,   0,     0,     0,   0,  0,  0;
       x*y,   0,     0,     y2+x12,x1*y1, f*x1,f*y,0,  0;
       0,     x*y,   0,     x1*y1, y2+y12,f*y1,0,  f*y,0;
       0,     0,     0,     f*x1,  f*y1,  f*f, 0  ,0,  0;
       f*x,   0,     0,     f*y,   0,     0,   f*f,0,  0;
       0,     f*x,   0,     0,     f*y,   0,   0,  f*f,0;
       0,0,0,               0,0,0,             0,0,0];
end

function epsil = get_eps(x_hat,x_hat1,dx,dx1,f)
    x    = x_hat(1) ; y    = x_hat(2);
    x1   = x_hat1(1); y1   = x_hat1(2);
    x_t  = dx(1)    ; y_t  = dx(2);
    x1_t = dx1(1)   ; y1_t = dx1(2);
    
    epsil = [x*x1 + x1*x_t + x*x1_t;
             x*y1 + y1*x_t + x*y1_t;
             (x   + x_t)*f         ;
             y*x1 + x1*y_t + y*x1_t;
             y*y1 + y1*y_t + y*y1_t;
             (y   + y_t)*f         ;
             (x1  + x1_t)*f        ;
             (y1  + y1_t)*f        ;
             f*f                   ];
end
