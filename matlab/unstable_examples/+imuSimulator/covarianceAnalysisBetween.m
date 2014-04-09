import gtsam.*;

% Test GTSAM covariances on a graph with betweenFactors
% Authors: Luca Carlone, David Jensen
% Date: 2014/4/6

clc
clear all
close all

%% Configuration
useAspnData = 1; % controls whether or not to use the ASPN data for scenario 2 as the ground truth traj

%% Create ground truth trajectory
trajectoryLength = 100;
unsmooth_DP = 0.5; % controls smoothness on translation norm
unsmooth_DR = 0.1; % controls smoothness on rotation norm

gtValues = Values;
gtGraph = NonlinearFactorGraph;

if useAspnData == 1
  sigma_ang = 1e-4;
  sigma_cart = 40;
else
  sigma_ang = 1e-2;
  sigma_cart = 0.1;
end
noiseVector = [sigma_ang; sigma_ang; sigma_ang; sigma_cart; sigma_cart; sigma_cart];
noise = noiseModel.Diagonal.Sigmas(noiseVector);

if useAspnData == 1
% Create a ground truth trajectory using scenario 2 data
  fprintf('\nUsing Scenario 2 ground truth data\n');
  % load scenario 2 ground truth data
  gtScenario2 = load('truth_scen2.mat', 'Lat', 'Lon', 'Alt', 'Roll', 'Pitch', 'Heading');
    
  % Add first pose
  currentPoseKey = symbol('x', 0);
  initialPosition = imuSimulator.LatLonHRad_to_ECEF([gtScenario2.Lat(1); gtScenario2.Lon(1); gtScenario2.Alt(1)]);
  initialRotation = [gtScenario2.Roll(1); gtScenario2.Pitch(1); gtScenario2.Heading(1)];
  currentPose = Pose3.Expmap([initialRotation; initialPosition]); % initial pose
  gtValues.insert(currentPoseKey, currentPose);
  gtGraph.add(PriorFactorPose3(currentPoseKey, currentPose, noise));
  prevPose = currentPose;
  
  % Limit the trajectory length
  trajectoryLength = min([length(gtScenario2.Lat) trajectoryLength]);
  
  for i=2:trajectoryLength
    currentPoseKey = symbol('x', i-1);
    gtECEF = imuSimulator.LatLonHRad_to_ECEF([gtScenario2.Lat(i); gtScenario2.Lon(i); gtScenario2.Alt(i)]);
    gtRotation = [gtScenario2.Roll(i); gtScenario2.Pitch(i); gtScenario2.Heading(i)];
    currentPose = Pose3.Expmap([gtRotation; gtECEF]);
        
    % Generate measurements as the current pose measured in the frame of
    % the previous pose
    deltaPose = prevPose.between(currentPose);
    gtDeltaMatrix(i-1,:) = Pose3.Logmap(deltaPose);
    prevPose = currentPose;
        
    % Add values
    gtValues.insert(currentPoseKey, currentPose);
       
    % Add the factor to the factor graph
    gtGraph.add(BetweenFactorPose3(currentPoseKey-1, currentPoseKey, deltaPose, noise));
  end
else
% Create a random trajectory as ground truth
  fprintf('\nCreating a random ground truth trajectory\n');
  % Add first pose
  currentPoseKey = symbol('x', 0);
  currentPose = Pose3; % initial pose
  gtValues.insert(currentPoseKey, currentPose);
  gtGraph.add(PriorFactorPose3(currentPoseKey, currentPose, noise));

  for i=1:trajectoryLength
    currentPoseKey = symbol('x', i);
    gtDeltaPosition = unsmooth_DP*randn(3,1) + [20;0;0]; % create random vector with mean = [1 0 0] and sigma = 0.5
    gtDeltaRotation = unsmooth_DR*randn(3,1) + [0;0;0]; % create random rotation with mean [0 0 0] and sigma = 0.1 (rad)
    gtDeltaMatrix(i,:) = [gtDeltaRotation; gtDeltaPosition];
    deltaPose = Pose3.Expmap(gtDeltaMatrix(i,:)');

    % "Deduce" ground truth measurements
    % deltaPose are the gt measurements - save them in some structure
    currentPose = currentPose.compose(deltaPose);
    gtValues.insert(currentPoseKey, currentPose);

     % Add the factors to the factor graph
    gtGraph.add(BetweenFactorPose3(currentPoseKey-1, currentPoseKey, deltaPose, noise));
  end
end
figure(1)
hold on;
plot3DTrajectory(gtValues, '-r', [], 1, Marginals(gtGraph, gtValues));
axis equal

numMonteCarloRuns = 10;
for k=1:numMonteCarloRuns
  fprintf('Monte Carlo Run %d.\n', k');
  % create a new graph
  graph = NonlinearFactorGraph;
  
  % noisy prior
  if useAspnData == 1
    currentPoseKey = symbol('x', 0);
    initialPosition = imuSimulator.LatLonHRad_to_ECEF([gtScenario2.Lat(1); gtScenario2.Lon(1); gtScenario2.Alt(1)]);
    initialRotation = [gtScenario2.Roll(1); gtScenario2.Pitch(1); gtScenario2.Heading(1)];
    initialPose = Pose3.Expmap([initialRotation; initialPosition] + (noiseVector .* randn(6,1))); % initial noisy pose
    graph.add(PriorFactorPose3(currentPoseKey, initialPose, noise));
  else
    currentPoseKey = symbol('x', 0);
    noisyDelta = noiseVector .* randn(6,1);
    initialPose = Pose3.Expmap(noisyDelta);
    graph.add(PriorFactorPose3(currentPoseKey, initialPose, noise));
  end
  
  for i=1:size(gtDeltaMatrix,1)
    currentPoseKey = symbol('x', i);
    
    % for each measurement: add noise and add to graph
    noisyDelta = gtDeltaMatrix(i,:)' + (noiseVector .* randn(6,1));
    noisyDeltaPose = Pose3.Expmap(noisyDelta);
    
    % Add the factors to the factor graph
    graph.add(BetweenFactorPose3(currentPoseKey-1, currentPoseKey, noisyDeltaPose, noise));
  end
  
  % optimize
  optimizer = GaussNewtonOptimizer(graph, gtValues);
  estimate = optimizer.optimize();
 
  figure(1)
  plot3DTrajectory(estimate, '-b');
  
  marginals = Marginals(graph, estimate);
  
  % for each pose in the trajectory
  for i=1:size(gtDeltaMatrix,1)+1
    % compute estimation errors
    currentPoseKey = symbol('x', i-1);
    gtPosition  = gtValues.at(currentPoseKey).translation.vector;
    estPosition = estimate.at(currentPoseKey).translation.vector;
    estR = estimate.at(currentPoseKey).rotation.matrix;
    errPosition = estPosition - gtPosition;
    
    % compute covariances:
    cov = marginals.marginalCovariance(currentPoseKey);
    covPosition = estR * cov(4:6,4:6) * estR';
    
    % compute NEES using (estimationError = estimatedValues - gtValues) and estimated covariances
    NEES(k,i) = errPosition' * inv(covPosition) * errPosition; % distributed according to a Chi square with n = 3 dof
  end

  figure(2)
  hold on
  plot(NEES(k,:),'-b','LineWidth',1.5)
end
%%
ANEES = mean(NEES);
plot(ANEES,'-r','LineWidth',2)
plot(3*ones(size(ANEES,2),1),'k--'); % Expectation(ANEES) = number of dof 
box on
set(gca,'Fontsize',16)
title('NEES and ANEES');

%%
figure(1)
box on
set(gca,'Fontsize',16)
title('Ground truth and estimates for each MC runs');

%% Let us compute statistics on the overall NEES
n = 3; % position vector dimension
N = numMonteCarloRuns; % number of runs
alpha = 0.01; % confidence level

% mean_value = n*N; % mean value of the Chi-square distribution 
% (we divide by n * N and for this reason we expect ANEES around 1)
r1 = chi2inv(alpha, n * N)  / (n * N);
r2 = chi2inv(1-alpha, n * N)  / (n * N);

% output here
fprintf(1, 'r1 = %g\n', r1);
fprintf(1, 'r2 = %g\n', r2);

figure(3)
hold on
plot(ANEES/n,'-b','LineWidth',2)
plot(ones(size(ANEES,2),1),'r-');
plot(r1*ones(size(ANEES,2),1),'k-.');
plot(r2*ones(size(ANEES,2),1),'k-.');
box on
set(gca,'Fontsize',16)
title('NEES normalized by dof VS bounds');

%% NEES COMPUTATION (Bar-Shalom 2001, Section 5.4)
% the nees for a single experiment (i) is defined as 
%               NEES_i = xtilda' * inv(P) * xtilda, 
% where xtilda in R^n is the estimation
% error, and P is the covariance estimated by the approach we want to test
% 
% Average NEES. Given N Monte Carlo simulations, i=1,...,N, the average
% NEES is:
%                   ANEES = sum(NEES_i)/N
% The quantity N*ANEES is distributed according to a Chi-square
% distribution with N*n degrees of freedom.
%
% For the single run case, N=1, therefore NEES = ANEES is distributed 
% according to a chi-square distribution with n degrees of freedom (e.g. n=3 
% if we are testing a position estimate)
% Therefore its mean should be n (difficult to see from a single run)
% and, with probability alpha, it should hold:
%
% NEES in [r1, r2]
%
% where r1 and r2 are built from the Chi-square distribution

