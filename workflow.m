%% Initialize Robot
rng('default'); % for repeatable result
dt = 0.05;      % Time step for simulation of the robot 
initialPose = [0  0  0  0]';
carbot = ExampleHelperCarBot(initialPose, dt);



%% Set up the Particle Filter
pf = robotics.ParticleFilter;

initialize(pf, 5000, [initialPose(1:3)', 0, 0, 0], eye(6), 'CircularVariables',[0 0 1 0 0 0]);
pf.StateEstimationMethod = 'mean';
pf.ResamplingMethod = 'systematic';

% StateTransitionFcn defines how particles evolve without measurement
pf.StateTransitionFcn = @BotStateTransition;

% MeasurementLikelihoodFcn defines how measurement affect the our estimation
pf.MeasurementLikelihoodFcn = @BotMeasurementLikelihood;

% Last best estimation for x, y and theta
lastBestGuess = [0 0 0];



%% Main Loop
% Run loop at 20 Hz for 20 seconds
r = robotics.Rate(1/dt); % The Rate object enables you to run a loop at a fixed frequency
reset(r);                % Reset the fixed-rate object

 % Reset simulation time
 simulationTime = 0;

 while simulationTime < 20 % if time is not up
    % Generate motion command that is to be sent to the robot
    % NOTE there will be some discrepancy between the commanded motion and the motion actually executed by the robot.
    uCmd(1) = 0.7*abs(sin(simulationTime)) + 0.1;  % linear velocity
    uCmd(2) = 0.08*cos(simulationTime);           % angular velocity
    drive(carbot, uCmd); % Move the robot forward  contaminate commanded motion with noise
    
    % Predict the carbot pose based on the motion model
    pf.Particles = BotStateTransition(pf.Particles,dt,uCmd);
    statePred = pf.Weights.'*pf.Particles;
%     Get GPS reading
%     The GPS reading is just simulated by adding Gaussian Noise to the truth data.
%     When the robot is in the roofed area, the GPS reading will not be available, the measurement will return an empty matrix.
   measurement = exampleHelperCarBotGetGPSReading(carbot); 

   % If measurement is available, then update, otherwise just use predicted result
   if ~isempty(measurement)
       pf.Weights = pf.Weights.*BotMeasurementLikelihood(pf.Particles, measurement);
       pf.Weights = pf.Weights/sum(pf.Weights);
       stateCorrected = pf.Weights.'*pf.Particles;
       N_eff = 1./sum(power(pf.Weights,2))/pf.NumParticles;
       if N_eff > 0.9
           %resample
           pf.Particles = datasample(pf.Particles,pf.NumParticles,'Weights',pf.Weights');
           pf.Weights = ones(pf.NumParticles,1)/pf.NumParticles;
       end
   else
       stateCorrected = statePred;  
   end

   lastBestGuess = stateCorrected(1:3);

   % Update plot
   if ~isempty(get(groot,'CurrentFigure')) % if figure is not prematurely killed
       updatePlot(carbot, pf, lastBestGuess, simulationTime);
   else
       break
   end

   waitfor(r);

   % Update simulation time
   simulationTime = simulationTime + dt;
end