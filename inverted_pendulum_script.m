%% Parameters
J = 1.91e-4;
m = 0.055;
g = 9.81;
l = 0.042;
b = 3e-6;
K = 0.0536;
R = 9.5;

%% Modelling
null_vector = zeros(1,1000);
staircaseSignal = timeseries(null_vector);
staircaseSignal.Data = reshape(staircaseSignal.Data, [1000,1]);
staircaseSignal.Time = reshape(staircaseSignal.Time, [1000, 1]);
staircaseSignal.Time = staircaseSignal.Time / 10;
staircaseSignal.Data(1:50,1,1) = 0;
staircaseSignal.Data(50:150,1,1) = 0.4;
staircaseSignal.Data(150:250,1,1) = 1;
staircaseSignal.Data(250:350,1,1) = 1.5;
staircaseSignal.Data(350:450,1,1) = -1.5;
staircaseSignal.Data(450:500,1,1) = -2;
staircaseSignal.Data(500:550,1,1) = -1;;
staircaseSignal.Data(550:600,1,1) = 0;
staircaseSignal.Data(600:650,1,1) = 1;
staircaseSignal.Data(650:750,1,1) = 3.5;
staircaseSignal.Data(750:1000,1,1) = 0; % I want to see how it stabilizes around 0.

%% MATLAB simulator of inverted pendulum by using relation (3)
Ts = 0.01;
Ad = [1, Ts; Ts*m*g*l/J, 1-Ts*(b+K^2/R)/J];
Bd = [0; Ts*K/R/J];

input = zeros(1,100);

x0 = [0.1; 0]; % initial state

% Get trajectory
x_traj = simulator_pendulum(x0, input, Ad, Bd);
% Plot trajectory
figure;
subplot(211);
plot(x_traj(1,:)); grid; title('Pozitia unghiulara');
subplot(212);
plot(x_traj(2,:)); grid; title('Viteza unghiulara');

%% Part 2: State - Feedback Control
Ac = [0 1;m*g*l/J (-b+K^2/R)/J];
Bc = [0; K/R/J];

% eig(Ac) = > s1 = -10.1368; s2 = 11.7044
% => Mut ambii poli in -1.
p = [-1, -1];
K_Gain = acker(Ac,Bc,p);
A_state_feedback = Ac-Bc*K_Gain;
disp(eig(A_state_feedback)); % ambii poli in -1.

%% Part 3: State Observer

Cd = [1, 0]; % only the position.
observer_poles = [0.1, 0.2]; % inside the unit circle.
L = place(Ad', Cd', observer_poles)';
A_observer = Ad-L*Cd;
disp(eig(A_observer)); % Se poate observa ca sunt in cercul unitate.