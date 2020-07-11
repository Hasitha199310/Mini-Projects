clear all;
close all;
% Kalman filter simulation for a robot travelling along a circular path.
% This robot has constant velocity and constant steering angle.
% INPUTS theta
% duration = length of simulation (seconds)
% dt = step size (seconds)
duration = 40;
dt = 0.01;
gpsmeasnoise = 1.5;         % position measurement noise (cm)
thetameasnoise = 1;     % position measurement noise (cm)
accelnoise = 0.01;         % acceleration noise (cm/sec^2)

v = 2.5;    %cosntant veloctiy
L = 1.5;      %Length L
phi=0.1;    %steeerig angle 


A = [1 0 0; 0 1 0; 0 0 1];       % transition matrix
B = [v*dt 0 0; 0 v*dt 0; 0 0 (v/L)*dt*tan(phi)];    % input matrix
H = [1 0 0; 0 1 0; 0 0 1];       % measurement matrix
x = [0; 0; 0];                   % initial state vector
xhat = x;                        % initial state estimate


R =[ gpsmeasnoise^2 0 0; 0 gpsmeasnoise^2 0; 0 0 thetameasnoise^2];   % measurement error covariance
Q = accelnoise^2 * [1 0 0; 0 1 0; 0 0 1];                             % process noise covariance
P = Q;                                                                % initial estimation covariance

% Initialize arrays for later plotting.
posx = [];       % true position array
posy = [];       % true position array
theta =[];       % true theta array

posxhat = [];    % estimated position array
posyhat = [];    % estimated position array
thetahat =[];    % estimated theta array

posxmeas = [];   % measured position array
posymeas = [];   % measured position array
thetameas=[];    % measured theata array


for t = 0 : dt: duration

u = [cos(x(3)); sin(x(3)); 1]; %input vector

% Process modle of the linear system with noise.
ProcessNoise = accelnoise * [(dt^2/2)*randn; (dt^2/2)*randn; dt*randn];
x = A * x + B * u + ProcessNoise;

% Measurement model with noise
MeasNoise = [gpsmeasnoise * randn; gpsmeasnoise * randn; thetameasnoise * randn];
y = H * x + MeasNoise;

%%%%  Kalman filter equations %%%%

% Time update equations
xhat = A * xhat + B * u;
P=A*P*A'+Q;

%%%% Measurement Update equations

% Form the Innovation vector.
Inn = y - H * xhat;
% Compute the covariance of the Innovation.
s = H * P * H' + R;
% Form the Kalman Gain matrix.
K = P * H' * inv(s);
% Update the state estimate.
xhat = xhat + K * Inn;
% Compute the covariance of the estimation error.
n=length(K*H);
P=(eye(n)-K*H)*P;

% Save some parameters for plotting later.
posx = [posx; x(1)];
posy = [posy; x(2)];
theta =[theta;x(3)];

posxmeas = [posxmeas; y(1)];
posymeas = [posymeas; y(2)];
thetameas=[thetameas;y(3)];

posxhat = [posxhat; xhat(1)];
posyhat = [posyhat; xhat(2)];
thetahat =[thetahat;xhat(3)];
end

% Plot the results
close all;
t = 0 : dt : duration;
figure;
plot(posx,posy,'-b.', posxmeas,posymeas,'r.', posxhat,posyhat,'-g.');
legend('True','Measured', 'Estimated');
grid;
xlabel('Position X/(cm)');
ylabel('Position Y/(cm)');
title('Position variation of the robot')

figure;
plot(t,theta,'b.',t,thetameas,'r.',t,thetahat,'g.')
xlabel('Time/s')
ylabel('theta(radian)')
title('Orientation variation of the robot')
legend('true','Measured','Estimated')

figure;
subplot(311)
plot(t,posx-posxmeas,'-b.',t,posx-posxhat,'-r.')
xlabel('Time/sec')
ylabel('Error-X/(cm)')
title('State measurment error and State estimation error')
legend('X-E mes','X-E est')

subplot(312)
plot(t,posy-posymeas,'-b.',t,posy-posyhat,'-r.')
xlabel('Time/sec')
ylabel('Error-Y/(cm)')
legend('Y-E mes','Y-E est')

subplot(313)
plot(t,theta-thetameas,'-b.',t,theta-thetahat,'-r.')
xlabel('Time/sec')
ylabel('Error-theta/(radian)')
legend('theta-E mes','theta-E est')


