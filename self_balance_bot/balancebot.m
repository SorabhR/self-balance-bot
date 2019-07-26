g = 9.8;
mw = 0.3;% wheel mass
mb = 0.6; % bot mass
rw = 0.03; % radius of wheel
rb = 1; % radius of bot
l = 0.158; % COG
Ib = mb*l*l;
Iw = (1/2)*mw*rw*rw;
den = mb*l*l+Ib;
value = mb*g*l/den;
x = rw * (mb+2*mw+((2*Iw)/(rw*rw)));



A = [0,1,0,0;
    0,0,0,0;
    0,0,0,1;
    0,0,value,0];

B = [0,0;
    -1/x,-1/x;
    0,0;
    -1/den,-1/den];


Q = [1,0,0,0;
    0,1,0,0;
    0,0,100,0;
    0,0,0,1];

R = [1,0;0,1];

K = lqr(A,B,Q,R)
