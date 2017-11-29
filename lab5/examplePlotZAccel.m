


%% USE THIS TO PLOT TIME HISTORY OF THE VEHICLE'S ANGLE AND DESIRED ANGLE AGAINST TIME
%% USES DEBUGVAL1 AS PITCH ATTITUDE ESTIMATE, USES DEBUGVAL9 AS ANGLE SETPOINT
%% THESE WERE SET ACCORDING TO DIRECTIONS IN LAB 4 UNDER SECTION 4.4



close all;

%% Deliverable 2:
% %% Generate pitch and roll estimate graph;
% figure;
% name = 'del2.csv'; %% NAME OF LOGFILE (CHANGE IF DIFFERENT)
% log = csv2struct(name, 29);
% hold off;
% plot(log.time, log.debugval0, 'color', 'r'); %estRoll
% hold on;
% plot(log.time, log.debugval1, 'color', 'b');  %estPitch
% 
% xlabel('Time (s)');
% ylabel({'Roll Estimate (rad) (red)', 'Pitch Estimate (rad) (blue)'});
% str = char([name(1:end-4) '_roll_pitch_est.jpg']);
% saveas(gcf, str);

% %% Generate vertical velocity graph:
% figure;
% plot(log.time, log.debugval5); %estVelocity_3
% xlabel('Time (s)');
% ylabel('Vertical Velocity (m/s)');
% str = char([name(1:end-4) '_vert_vel.jpg']);
% saveas(gcf, str);
% 
% %% Generate height estimate and distance sensor measurement graph:
% figure;
% hold off;
% plot(log.time, log.debugval6, 'color', 'r'); %height estimate
% hold on;
% plot(log.time, log.heightSensor, 'color', 'b');  %height sensor
% 
% xlabel('Time (s)');
% ylabel({'Height estimate (m) (red)', 'Height sensor (m) (blue)'});
% 
% str = char([name(1:end-4) '_heightEst_heightSens.jpg']);
% saveas(gcf, str);
% 
% 
% %% Deliverable 3:
%% Generate rate gyro measurements graph:
figure;
name = 'del3.csv'; %% NAME OF LOGFILE (CHANGE IF DIFFERENT)
log = csv2struct(name, 29);

hold off;
plot(log.time, log.rateGyro.x, 'color', 'r'); %rateGyro roll
hold on;
plot(log.time, log.rateGyro.y, 'color', 'g'); %rateGyro pitch 
hold on;
plot(log.time, log.rateGyro.z, 'color', 'b'); %rateGyro yaw 

title('Rate Gyro Roll, Pitch, Yaw  Measurements');
xlabel('Time (s)');
ylabel({'Roll (rad/s) (Red)', 'Pitch (rad/s) (Green)', 'Yaw (rad/s) (Blue)'});

str = char([name(1:end-4) '_rate_gyro_xyz_meas.jpg']);
saveas(gcf, str);
% 
% %% Generate output from flow sensor (x, y) graph:
% figure;
% hold off;
% plot(log.time, log.opticalflow.x, 'color', 'r'); %optical flow x
% hold on;
% plot(log.time, log.opticalflow.y, 'color', 'b');  %optical flow y
% 
% xlabel('Time (s)');
% ylabel({'Optical flow x (rad/s) (red)', 'Optical flow y (rad/s) (blue)'});
% 
% str = char([name(1:end-4) '_opticalFlow_x_y.jpg']);
% saveas(gcf, str);
% 
% %% Generate horizontal velocity (x, y) graph:
% figure;
% hold off;
% plot(log.time, log.debugval3, 'color', 'r'); %estimated horiz vel x
% hold on;
% plot(log.time, log.debugval4, 'color', 'b');  %%estimated horiz vel y
% 
% xlabel('Time (s)');
% ylabel({'Estimated Horizontal Velocity x (m/s) (red)', 'Estimated Horizontal Velocity y (m/s) (blue)'});
% 
% 
% str = char([name(1:end-4) '_horizontalVel_x_y.jpg']);
% saveas(gcf, str);
% 
% %% Generate height estimate  graph:
% figure;
% plot(log.time, log.debugval6); %height estimate
% 
% xlabel('Time (s)');
% ylabel('Height estimate (m)');
% str = char([name(1:end-4) '_heightEst.jpg']);
% saveas(gcf, str);

% %% Generate debug data graphs
% figure;
% name = './smooth_log/log___2017_11_28___19_38_40_latest_good file.csv'; %% NAME OF LOGFILE (CHANGE IF DIFFERENT)
% log = csv2struct(name, 29);
% hold off;
% plot(log.time, log.debugval10, 'color', 'r'); %estPos1
% hold on;
% plot(log.time, log.debugval11, 'color', 'b');  %estPos2
% 
% xlabel('Time (s)');
% ylabel({'Pos Estimate 1(m) (red)', 'Pos Estimate 2(m) (blue)'});
% str = char([name(1:end-4) '_pos_1_2_est.jpg']);
% saveas(gcf, str);


% example plotting data using csv2struct() function



% This saves all data in the above struct 's_speed1000'. 
% Now you can plot z-accelerometer output:
% close all;
% for i = 1:5
%     name = ['gyro-Log' int2str(i) '.csv'];
%     log = csv2struct(name, 29);
%     yyaxis left
%     plot(log.time, log.debugval7); %pitch angle estimate
%     yyaxis right
%     plot(log.time, log.debugval4);  %rate gyro data corrected
%     
%     yyaxis left
%     title(['Pitch Angle Estimate, Y Rate Gyro Data Corrected ' int2str(i)]);
%     xlabel('Time (s)');
%     ylabel('Pitch Angle Estimate');
%     
%     yyaxis right;
%     ylabel('Y Rate Gyro Data Corrected');
%     str = char([name(1:end-4) '.jpg']);
%     saveas(gcf, str);
% end



% close all;
% for i = 1:5
%     name = ['3Dgyro-Log' int2str(i) '.csv'];
%     log = csv2struct(name, 29);
%     hold off;
%     plot(log.time, log.debugval0, 'color', 'r'); %roll attitude estimate
%     hold on;
%     plot(log.time, log.debugval1, 'color', 'g'); %pitch attitude estimate
%     hold on;
%     plot(log.time, log.debugval2, 'color', 'b'); %yaw attitude estimate
%     
%     title(['Roll, Pitch, Yaw Attitude Estimates ' int2str(i)]);
%     xlabel('Time (s)');
%     ylabel({'Roll (Red)', 'Pitch (Green)', 'Yaw (Blue)'});
%     
%     str = char([name(1:end-4) '.jpg']);
%     saveas(gcf, str);
% end