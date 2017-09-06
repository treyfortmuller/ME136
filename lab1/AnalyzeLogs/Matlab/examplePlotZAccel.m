% example plotting data using csv2struct() function

% assuming your csv file name is log_file_speed1000.csv with 28 columns:
s_speed1000_off = csv2struct('motor-off.csv', 29);
s_speed1000_on = csv2struct('motor-on.csv', 29);

% This saves all data in the above struct 's_speed1000'. 
% Now you can plot z-accelerometer output:
figure, plot(s_speed1000_off.time, s_speed1000_off.rateGyro.x);
title('Motor Off, Rate Gyroscope Output: x');
xlabel('Time (s)');
ylabel('Angualr Velocity (rad/s)');
figure, plot(s_speed1000_off.time, s_speed1000_off.rateGyro.y);
title('Motor Off, Rate Gyroscope Output: y');
xlabel('Time (s)');
ylabel('Angualr Velocity (rad/s)');
figure, plot(s_speed1000_off.time, s_speed1000_off.rateGyro.z);
title('Motor Off, Rate Gyroscope Output: z');
xlabel('Time (s)');
ylabel('Angualr Velocity (rad/s)');
figure, plot(s_speed1000_on.time, s_speed1000_on.rateGyro.x);
title('Motor On, Rate Gyroscope Output: x');
xlabel('Time (s)');
ylabel('Angualr Velocity (rad/s)');
figure, plot(s_speed1000_on.time, s_speed1000_on.rateGyro.y);
title('Motor On, Rate Gyroscope Output: y');
xlabel('Time (s)');
ylabel('Angualr Velocity (rad/s)');
figure, plot(s_speed1000_on.time, s_speed1000_on.rateGyro.z);
title('Motor On, Rate Gyroscope Output: z');
xlabel('Time (s)');
ylabel('Angualr Velocity (rad/s)');