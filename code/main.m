
clear all;
close all;
%% Plot the path of the ball
load ../data_files/training_1.mat
figure(1);
clf;
plot(ball(1, :), ball(2, :), 'bo-');
hold on;
plot(ball(1, end), ball(2, end), 's', ...
    'MarkerSize', 10, 'MarkerEdgeColor', [.5 0 0], 'MarkerFaceColor', 'r');
plot(ball(1, 1), ball(2, 1), 's', ...
    'MarkerSize', 10, 'MarkerEdgeColor', [0 .5 0], 'MarkerFaceColor', 'g');
hold off;
axis equal;
title('Ball Position tracking');
xlabel('X (m)');
ylabel('Y (m)');

%% Algorithm
state = [0,0,0,0];
last_t = -1;
N = numel(t);
myPredictions = zeros(2, N);
param = {};
for i=1:N
    [ px, py, state, param ] = kalmanFilter( t(i), ball(1,i), ball(2,i), state, param, last_t);
    if numel(state)~=4
        error('Your state should be four dimensions.');
    end
    last_t = t(i);
    myPredictions(1, i) = px;
    myPredictions(2, i) = py;
end
clear px py;

%% Overlay the predictions
figure(1);
hold on;
plot(myPredictions(1, :), myPredictions(2, :), 'k+-');
hold off;

%% Show the error
nSkip = 14;
myError = myPredictions(:, 1:end-nSkip) - ball(:, 1+nSkip:end);
myError_dist = sqrt(myError(1,:).^2 + myError(2,:).^2);
myError_mean = mean(myError_dist);
figure(2);
clf;
plot(myError_dist);
title('Prediction Error Over Time');
xlabel('Frame');
xlim([1, numel(myError_dist)]);
ylabel('Error (m)');
legend(sprintf('Prediction: %.2f mean', myError_mean));

%% Load the solution
load ../data_files/test_1.mat

% Error
error = predictions(:, 1:end-nSkip) - ball(:, 1+nSkip:end);
error_dist = sqrt(error(1,:).^2 + error(2,:).^2);
error_mean = mean(error_dist);
figure(2);
hold on;
plot(error_dist);
hold off;
legend(sprintf('Prediction: %.2f mean', myError_mean),...
    sprintf('Kalman Prediction: %.2f mean', error_mean));

figure(1);
hold on;
plot(predictions(1, :), predictions(2, :), 'mo-');
hold off;
legend('Observed','End','Start','Prediction','Kalman Prediction');

