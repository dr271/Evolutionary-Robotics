%A test fitness function when trying to evolve a 'return to home' behaviour
%Evolves ctrnn weights to return any given static goalOutput

%INPUT: Genotype
%OUTPUT: Cost of genotype, denoted by discrepancy between output nodes
%value and goalOutput.

function [error] = EvolTest(W)
outputs = [];
iterations = 5
goalOutput = 0.314159262
errors = [];
for j = 1:iterations %How many iterations to average each genotype over
    
    T = 8; %Run time
    pos = [0;0]; %Initial position
    bearing = 0; %Initial bearing
    plot_flag = true;

    dt=0.5; %timestep size
    R =0.05; %radius of agent
    b = 45; %(degrees) sensor angles
    % Initial setup
    %no nodes
    N = 5;
    %set weights randonly
    %W = randn(N)*.2;
    %initial conditions
    time= 0:dt:T;
    n=zeros(N,length(time));
    n(1,1) = -1;
    n(2,1) = 1;
    %biases are zeros to start
    theta = floor(rand(N,1)+0.5)*0.0;

    motor_gain =1;
    vl=0;vr=0;
    %convert to radians
    bearing = bearing/360*2*pi;b=b/360*2*pi; 


    for t=2:length(time)

        %Forwards velocity
        vf = (vl+vr)./2;
        %Angular velocity
        va = (vr-vl)./(2.*R);

        %Movement
        pos(1,t) = pos(1,t-1) + dt*vf.*cos(bearing(1,t-1));%X component
        pos(2,t) = pos(2,t-1) + dt*vf.*sin(bearing(1,t-1));%Y component
        bearing(1,t) = mod(bearing(1,t-1) + dt*va,2*pi);%Bearing update
        %Random motor speed update
        lm = rand;
        rm = rand;

        %Scale by motor gains
        vl =motor_gain.*lm; %add mosie
        vr =motor_gain.*rm;

        %Continuous Time-recurrent Neural Network
        I(5,1) = vl*10;%input
        I(4,1) = vr*10;%input
        n(:,t) = n(:,t-1) +dt*(-n(:,t-1)+tanh( W*n(:,t-1) +theta+I));%ctrnn

    end
    plot(time,n)


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


  

    %Retrieve output
    outputNode = n(2,length(time));
    disp("Output Node: " + outputNode)

    outputs = [outputs outputNode];

    
    
    %Error calculation (fitness function)
    if (outputNode < goalOutput)
        errors = [errors (goalOutput - outputNode)];
    elseif (outputNode >= goalOutput)
        errors = [errors (outputNode - goalOutput)];
    end
    disp("Error: " + errors(j));

    
end %of genotype average loop

error = mean(errors);
disp("Average error: " + error + " over " + iterations + " iterations per genotype");

end %of func






