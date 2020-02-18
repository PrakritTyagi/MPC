function h = ex01StepPlotFunction(sysList,log,plot_handles,k)
clf
logX = log{1}.stateTrajectory(:,1:k);hold on;
h    = plot3(logX(1,:),logX(2,:),logX(3,:));
h    = plot(logX(6,:),logX(7,:));

%cylinderical obstacle
a0 = 180;
b0 = 25;
a1 = 400;
b1 = 110;
a2 = 495;
b2 = 50;
r = 20;

th = 0:pi/50:2*pi;

xunit = r * cos(th) + a0;
yunit = r * sin(th) + b0;
xunit1 = r * cos(th) + a1;
yunit1 = r * sin(th) + b1;
xunit2 = r * cos(th) + a2;
yunit2 = r * sin(th) + b2;
h = plot(xunit, yunit);
h = plot(xunit1,yunit1);
h = plot(xunit2,yunit2);

%Camera Parameters and plotting FOV
VFOV_deg = 94.4;
HFOV_deg = 122.6;
VFOV_rad = VFOV_deg*pi/180;
HFOV_rad = HFOV_deg*pi/180;


a =  logX(3,end)*tan(VFOV_rad/2);
b =  logX(3,end)*tan(HFOV_rad/2);

h = plot(logX(1,end)+a*sin(th),logX(2,end)+b*cos(th));

%labels
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');
hold off
axis equal;
grid on 
%camera_matlab
camva auto;
cameratoolbar;

end
