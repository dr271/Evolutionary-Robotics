%First objective function (two-eyed phototaxis)
%Call this function from the command window to simulate a genotype for the
%given run time

%INPUT: Genotype
%OUTPUT: Cost of genotype, denoted by agents distance from light source
%(at [0,0]) after run time expires.

function [distFromLight] = pTaxisDist(geno)

T = 40;         %Run time (seconds)
pos = [1;1];    %Initial position
bearing = 90;   %Initial bearing
plot_flag = true; %Plot path
%maxVelo = 0;   %Used to log variables max an min values for reality gap scaling
%minVelo = 0;

dt=0.05;    %Time-step size
R =0.05;	%Agent radius
b = 45;     %sensor angles (degrees)

%conver geno params
w_ll = geno(1); 
w_lr = geno(2);
w_rl = geno(3);
w_rr = geno(4);
bl = geno(5);
br = geno(6);

rho=zeros(2,1);
sensor_gain = 1;
motor_gain =1;
vl=0;vr=0;
%convert to radians
bearing = bearing/360*2*pi;b=b/360*2*pi;


for t=2:floor(T/dt)

    %Forwards velocity
    vc = (vl+vr)./2;

    %Angular Velocity
    va = (vr-vl)./(2.*R);

    %Update X, Y and bearing
    pos(1,t) = pos(1,t-1) + dt*vc.*cos(bearing(1,t-1));
    pos(2,t) = pos(2,t-1) + dt*vc.*sin(bearing(1,t-1));
    bearing(1,t) = mod(bearing(1,t-1) + dt*va,2*pi);

    % Calculate left sensor position
    sl_pos(1,1) = pos(1,t) + R.*cos(bearing(1,t)+b);
    sl_pos(2,1) = pos(2,t) + R.*sin(bearing(1,t)+b);

    % Calculate right sensor position
    sr_pos(1,1) = pos(1,t) + R.*cos(bearing(1,t)-b);
    sr_pos(2,1) = pos(2,t) + R.*sin(bearing(1,t)-b);

    % Calculate (square) distance to element
    dl = sqrt((sl_pos(1,1)).^2+(sl_pos(2,1)).^2);
    dr = sqrt((sr_pos(1,1)).^2+(sr_pos(2,1)).^2);

    % Calculate local intensity
    il = sensor_gain/dl;
    ir = sensor_gain/dr;

    %Linear funtion mapping motor speed to sensor readings
    lm = il.*w_ll + ir.*w_rl + bl; %LeftMotor = IntensityLeft*Weight1 + IntensityRight*Weight3 + Bias1
    rm = il.*w_lr + ir.*w_rr + br; %RightMotor = IntensityLeft*Weight2 + IntensityRight*Weight4 + Bias2

    % Scale by motor gains
    vl =motor_gain.*lm;
    vr =motor_gain.*rm;
    
    %Used to log variables max an min values for reality gap scaling
%     if (lm > maxVelo)
%         maxVelo = lm;
%     end
%     if (rm < minVelo)
%         minVelo = rm;
%     end

end

if(plot_flag)

    %final postion
    x=pos(1,end);
    y= pos(2,end);
    f_bearing = bearing(1,end);

    % Calculate left sensor position
    sl_pos(1,1) = x + R.*cos(f_bearing+b);
    sl_pos(2,1) = y + R.*sin(f_bearing+b);

    % Calculate left sensor position
    sr_pos(1,1) = x + R.*cos(f_bearing-b);
    sr_pos(2,1) = y + R.*sin(f_bearing-b);

    %plot agent
    clf;
    %plot light
    plot(0,0,'.','MarkerSize',30,'Color','y');hold on;
    plot(0,0,'ko','MarkerSize',10);

    % Plot left sensor
    plot([x,sl_pos(1,1)],[y,sl_pos(2,1)],'k');
    plot(sl_pos(1,1),sl_pos(2,1), ...
        '.','Markersize',20,'Color','r');

    % Plot left sensor
    plot([x,sr_pos(1,1)],[y,sr_pos(2,1)],'k');
    plot(sr_pos(1,1),sr_pos(2,1), ...5
        '.','Markersize',20,'Color','r');
    % Plot body
    plot( x, y,'.','Markersize',40,'Color','b');
    plot(x,y,'ko','Markersize',14);
    % Plot trajkectory
    plot(pos(1,:),pos(2,:),'k');
    LIMITS   = [-max(max(abs(pos)))-0.1, max(max(abs(pos)))+0.1];
    axis([LIMITS,LIMITS]);

end
 
distFromLight = sqrt(pos(1,end)^2 + pos(2,end)^2);
disp("Average Dist From Light: " + distFromLight);