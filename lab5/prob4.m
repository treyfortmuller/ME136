dt = 1/500;
num_cycles = 5*(1/dt);

AngEst = 0; %21
AngEst_Vals = zeros(num_cycles, 1);
AngVelEst = 0; %22

Ang = pi/3; %23

Ang_Vals = zeros(num_cycles, 1);
AngVel = 0; %24
AngVel_Vals = zeros(num_cycles, 1);

AngCmd = 0;
AngAccCmd = 0;
AngAccCmd_Vals = zeros(num_cycles, 1);

dt = 1/500;
timeConst_pitchAngle = 0.12;
timeConst_pitchRate = 0.04;
g = 9.81;
p = 0.01;


num_cycles = 5*(1/dt);

t = 0;
t_Vals = (1:num_cycles)';

while (t < num_cycles)
    % simulated sensors:
    rateGyro = AngVel; %19
    Acc = -g*sin(Ang); %20

    % estimator:
    AngVelEst = rateGyro; %15
    AngPred = AngEst + AngVelEst*dt; %16
    AngMeas = -Acc/g; %17
    
    % commanded:
    AngVelCmd = (-1/timeConst_pitchAngle)*(AngEst-AngCmd); %13
    AngEst_Vals(t+1) = AngEst;
    AngEst = (1-p)*AngPred + p*AngMeas; %18
    
    AngAccCmd = (-1/timeConst_pitchRate)*(AngVelEst-AngVelCmd); %14
    AngAccCmd_Vals(t+1) = AngAccCmd;
    
    % k+1:
    Ang_Vals(t+1) = Ang;
    Ang = Ang + AngVel*dt + (1/2)*AngAccCmd*dt^2; %11
    AngVel_Vals(t+1) = AngVel;
    AngVel = AngVel + AngAccCmd*dt; %12
    
    t = t + 1;
end
  
figure;
hold off;
plot(t_Vals, Ang_Vals, 'color', 'r');
hold on;
plot(t_Vals, AngEst_Vals, 'color', 'b'); 

xlabel('Time (ms)');
ylabel({'Angle(rad) (red)', 'Angle Est(rad) (blue)'});
str = 'ang_and_ang_est_graph.png';
saveas(gcf, str);


figure;
plot(t_Vals, AngVel_Vals, 'color', 'b'); 

xlabel('Time (ms)');
ylabel({'Angle Velocity(rad/s)'});
str = 'ang_vel_graph.png';
saveas(gcf, str);


figure;
plot(t_Vals, AngAccCmd_Vals, 'color', 'b'); 

xlabel('Time (ms)');
ylabel({'Angle Acceleration Commanded(rad/s^2)'});
str = 'ang_acc_cmd_graph.png';
saveas(gcf, str);