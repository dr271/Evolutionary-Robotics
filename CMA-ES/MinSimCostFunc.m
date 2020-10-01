%The cost function for the 'return to home' behaviour
%INPUT: Genotype
%OUTPUT: Discrepancy between predicted home bearing and actual

function [error] = MinSimCostFunc(genotype)
    errors = [];
    
    %Separate network weights and biases
    W = genotype(1:6,1:6);
    theta = genotype(1:6,7);
    
    
    for j = 1:50        %How many iterations to average each genotype over
        T = 8;          %Run time
        pos = [0;0];    %Initial position
        bearing = 0;    %Initial bearing
        plot_flag = false;
 
        dt=0.01; %timestep size
        R =0.05; %radius of agent
        b = 45; %(degrees) sensor angles
        
        %Initial setup
        %no nodes
        N = 6;
        %set weights randonly
        %W = randn(N)*.2;
        %initial conditions
        time= 0:dt:T;
        n=randn(N,length(time));
%         n(1,1) = -1;
%         n(2,1) = 1;
        %biases are zeros to start
        theta = floor(rand(N,1)+0.5)*0.0;
 
        motor_gain =5;
        vl=0;vr=0;
        %convert to radians
        bearing = bearing/360*2*pi;b=b/360*2*pi; 
 
 
        
        smComl(1) = 0;
        smComr(1) = 0;
        
        for t=2:length(time)
 
            %Forwards velocity
            vf = (vl+vr)./2;
            %Angular velocity
            va = (vr-vl)./(2.*R);
 
            %Movement
            pos(1,t) = pos(1,t-1) + dt*vf.*cos(bearing(1,t-1));%X component
            pos(2,t) = pos(2,t-1) + dt*vf.*sin(bearing(1,t-1));%Y component
            bearing(1,t) = mod(bearing(1,t-1) + dt*va,2*pi);%Bearing update
            %Random motor speed update with smoothing
             smComl(t) =  smComl(t-1) +10*dt*(-smComl(t-1) +randn);
             smComr(t) =  smComr(t-1) +10*dt*(-smComr(t-1) +randn);
            
            lm = smComl(t);
            rm = smComr(t);
 
            %Scale by motor gains
            vl =motor_gain.*lm; %add mosie here
            vr =motor_gain.*rm;
 
            %Continuous Time-recurrent Neural Network
            I(6,1) = lm;%(lm-0.5)*10;%input
            I(5,1) = rm;%(rm-0.5)*10;%input
            I(4,1) = bearing(1,t);
            n(:,t) = n(:,t-1) +10*dt*( -n(:,t-1)+tanh( W*n(:,t-1) +I + theta)  );
            
            %ctrnn
 
            Iall(t) =  I(5,1);
        end
     %figure(7);   plot(time,n)
 
 
         %final postion
         x=pos(1,end);
         y= pos(2,end);
         f_bearing = bearing(1,end);
 
 
        if(plot_flag)
            %plot agent
            clf;
            % Plot body
            plot( x, y,'.','Markersize',40,'Color','b');
            plot(x,y,'ko','Markersize',14);
            % Plot trajectory
            plot(pos(1,:),pos(2,:),'k');
            LIMITS   = [-max(max(abs(pos)))-0.1, max(max(abs(pos)))+0.1];
            axis([LIMITS,LIMITS]);
        end
 
 
        %Calculate True Home Bearing
        if (x < pos(1,1) && y > pos(2,1))%TopLeft
                   %disp("TopLeft")
                   i = atan(abs(x/abs(y)));
                   actualHomeBearing = pi - i;
                   disp("Return Home Bearing: " + actualHomeBearing)
               elseif (x > pos(1,1) && y > pos(2,2))%TopRight
                   %disp("TopRight")
                   i = atan(abs(x/abs(y)));
                   actualHomeBearing = pi + i;
                   disp("Return Home Bearing: " + actualHomeBearing)
               elseif (x < pos(1,1) && y < pos(2,2))%BottomLeft
                   %disp("BottomLeft")
                   i = atan(abs(x/abs(y)));
                   actualHomeBearing = i;
                   disp("Return Home Bearing: " + actualHomeBearing)
               elseif (x > pos(1,1) && y < pos(2,2))%BottomRight
                   %disp("BottomRight")
                   i = atan(abs(x/abs(y)));
                   actualHomeBearing = pi*2 -i;
                   disp("Return Home Bearing: " + actualHomeBearing)
        end
 
 
        %Estimated Home Bearing
        estHomeBearing = (pi * (n(1,length(time)))) + pi;%2 is arbitrary output node, pi to scale to radians from tanh -1:1
        disp("Predicted home bearing: " + estHomeBearing)
        
        
        %Error calculation (fitness function)
        diff = abs(actualHomeBearing - estHomeBearing);
        if (diff <= pi)
            errors = [errors diff^10];
        else %No error can be > pi, accounts for wrap-around error calculation
            errors = [errors (min(actualHomeBearing, estHomeBearing) + (2*pi - max(actualHomeBearing, estHomeBearing)))^10];
        end
        disp("Squared Error: " + errors(j));
 
        
    end %of genotype average loop
 
    error = mean(errors);
    disp("AVERAGE SQUARED ERROR: " + error);
 
end %of func
