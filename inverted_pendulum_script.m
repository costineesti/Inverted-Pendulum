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
staircaseSignal.Data(50:150,1,1) = 2;
staircaseSignal.Data(150:250,1,1) = 4;
staircaseSignal.Data(250:350,1,1) = 7;
staircaseSignal.Data(350:450,1,1) = 10;
staircaseSignal.Data(450:500,1,1) = 7;
staircaseSignal.Data(500:550,1,1) = 4;;
staircaseSignal.Data(550:600,1,1) = 2;
staircaseSignal.Data(600:650,1,1) = 0;
staircaseSignal.Data(650:750,1,1) = -5;
staircaseSignal.Data(750:1000,1,1) = 0; % I want to see how it stabilizes around 0.