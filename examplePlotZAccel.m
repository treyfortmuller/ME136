


%% USE THIS TO PLOT TIME HISTORY OF THE VEHICLE'S ANGLE AND DESIRED ANGLE AGAINST TIME
%% USES DEBUGVAL1 AS PITCH ATTITUDE ESTIMATE, USES DEBUGVAL9 AS ANGLE SETPOINT
%% THESE WERE SET ACCORDING TO DIRECTIONS IN LAB 4 UNDER SECTION 4.4



close all;
NUM_LOG_FILES = 3; 
%% Generate pitch angle and desired angle graphs:
for i = 1:NUM_LOG_FILES 
    name = ['Agyro-Log' int2str(i) '.csv']; %% NAME OF LOGFILE (CHANGE IF DIFFERENT)
    log = csv2struct(name, 29);
    yyaxis left
    plot(log.time, log.debugval1); %pitch attitude estimate
    yyaxis right
    plot(log.time, log.debugval9);  %angle setpoint (30deg) = 0.5236rad when blue button pressed, 0 otherwise
    
    yyaxis left
    title(['Pitch Attitude Estimate and Angle Setpoint ' int2str(i)]);
    xlabel('Time (s)');
    ylabel('Pitch Attitude Estimate (rad/s)');
    
    yyaxis right;
    ylabel('Angle Setpoint (rad/s)');
    str = char([name(1:end-4) '.jpg']);
    saveas(gcf, str);
end

%% Generate Angular velocity and desired ang vel graphs:
for i = 1:NUM_LOG_FILES 
    name = ['Agyro-Log' int2str(i) '.csv']; %% NAME OF LOGFILE (CHANGE IF DIFFERENT)
    log = csv2struct(name, 29);
    yyaxis left
    plot(log.time, log.debugval4); %vehicle's pitch angular velocity = rategyro_corr.y
    yyaxis right
    plot(log.time, log.debugval7);  %desired angular velocity = cmdAngVel.y
    
    yyaxis left
    title(['Pitch Attitude Estimate and Angle Setpoint ' int2str(i)]);
    xlabel('Time (s)');
    ylabel('Pitch Attitude Estimate (rad/s)');
    
    yyaxis right;
    ylabel('Angle Setpoint (rad/s)');
    str = char([name(1:end-4) '.jpg']);
    saveas(gcf, str);
end


%% Generate motor command graphs:
for i = 1:NUM_LOG_FILES 
    name = ['Agyro-Log' int2str(i) '.csv']; %% NAME OF LOGFILE (CHANGE IF DIFFERENT)
    log = csv2struct(name, 29);
    hold off;
    plot(log.time, log.motorcmd1, 'color', 'r'); 
    hold on;
    plot(log.time, log.motorcmd2, 'color', 'g'); 
    hold on;
    plot(log.time, log.motorcmd3, 'color', 'b'); 
    hold on;
    plot(log.time, log.motorcmd4, 'color', 'y'); 
        
    title('Motor PWM Commands');
    xlabel('Time (s)');
    ylabel({'M1 (Red)', 'M2 (Green)', 'M3 (Blue)', 'M4 (Yellow)'});
    
    str = char([name(1:end-4) '.jpg']);
    saveas(gcf, str);
end







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