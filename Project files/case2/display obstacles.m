%cylinderical obstacle
figure(1);
hold on;
a0 = 140;
b0 = 65;
h0 = 45;
a1 = 260;
b1 = 100;
h1 = 45;
a2 = 220;
b2 = 160;
h2 = 45;
r = 20;
n = 500;

[X0,Y0,Z0] = cylinder(r);
[X1,Y1,Z1] = cylinder(r);
[X2,Y2,Z2] = cylinder(r);
X0 = X0 + a0; Y0 = Y0 + b0; Z0 = Z0*h0;
X1 = X1 + a1; Y1 = Y1 + b1; Z1 = Z1*h1;
X2 = X2 + a2; Y2 = Y2 + b2; Z2 = Z2*h2;
h = surf(X0,Y0,Z0);
h = surf(X1,Y1,Z1);
h = surf(X2,Y2,Z2);
