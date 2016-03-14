% Philip Twu and Magnus Egerstedt
% Fall 2010
% © 2010 by Georgia Institute of Technology. All rights reserved.
%
% function [sensingTable]=senseObstacles(x,theta,phi,delta,currTargetWP)
% @param x (N x 2) matrix of agent states, where row i gives the
%                  [x-position y-position] of agent i
% @param theta (N x 1) vector of offset angles for the agents' relative
%              coord frames.  Agent i's relative coord frame is simply the
%              normal coord frame rotated theta(i) CCW.
% @param phi (S x 1) vector of angles in which an agent can sense for 
%            obstaclesin its own realtive coord frame
% @param delta The max distance an agent can sense from its centroid
% @param currTargetWP The current target waypoint.  It will be used to
%                     determine which obstacle map to use.
% @return sensingTable (N x S) matrix of sensor reading for each agent.
%                       Element (i,j) gives the sensing result for agent i
%                       when looking at angle phi(j) in its own relative 
%                       coord frame.  Values can be 0 ~ delta if an
%                       obstacle is sensed, or inf if no obstacle sensed.
%
% Takes in information about the agent states and checks it against the
% obstacle bitmap to simulate sensing for obstacles.  Returns a table of
% agent sensor readings for each of the angles in 'phi', for each agent.
%
% NOTE: MATLAB flips the axis strangely when using imagesc and then
% plotting over it.
%
% Storage:                    Imagesc():
%  x  y-------->               y  x ------------>
%  |                           |
%  |                           |
%  V                           V
%
% We will try to use the storage axis as much as possible for computations.
% Therefore, given some pt in storage (X,Y) to plot, all calls to plot() or
% drawCircle() or text() needs to pass in X and Y in the reverse order, 
% i.e., plot(Y,X)

function [sensingTable]=senseObstacles(x,theta,phi,delta,currTargetWP)

%% Settings
% Sensing resolution settings
sensingResolution = 50; % The range reading will be in the set 
                       % {0:sensingResolution/delta:delta} and inf

% Obstacle Map settings
persistent obstacleMap obstacleMap2 obstacleRead obstacleMapSize
obstacleFilename = 'obstacles 1.jpg';
obstacleFilename2 = 'obstacles 2.jpg';

% Read in the obstacle map if not already done so
% The resulting obstacle map will have a value '1' if there is an obstacle,
% and '0' if there is not.
if(isempty(obstacleRead))
    obstacleMap = 1-round(rgb2gray(imread(obstacleFilename))./255);
    obstacleMap2 = 1-round(rgb2gray(imread(obstacleFilename2))./255);
    obstacleMapSize = size(obstacleMap);
    obstacleRead = 1;
end

% Implicitly find the number of agents
N = size(x,1);

% Implicitly find the number of sensing angles
S = length(phi);

% Let all sensed distance be inf (no obstacles) initially
sensingTable = ones(N,S)*inf;

for i=1:N % Get the obstacle sensing data for agent i
    for j=1:S % Check all the sensor angles
        searchVector = rotmatrix(theta(i)+phi(j))*[1;0];
        for k=0:delta/sensingResolution:delta % k is the distance from agent
            % Find the corresponding pixel to sample, at angle phi(j) from
            % the x axis within agent i's coord frame, which is itself
            % rotated an angle theta(i) from the standard coord frame
            px = x(i,1:2)' + k*searchVector;
            
            % Round the pixel value to make sure that it doesn't exceed
            % image bounds
            px = round(px);
            px = max(px,1); % Pixel values must start from one
            px(1) = min(px(1),obstacleMapSize(1));
            px(2) = min(px(2),obstacleMapSize(2));
            
            % Sense for obstacle at distance k and angle phi(j) of agent i
            if(currTargetWP <= 3)
                if(obstacleMap(px(1),px(2)))
                    sensingTable(i,j) = k; % Record the finding
                    break; % Stop checking this angle for this agent
                end
            else
                if(obstacleMap2(px(1),px(2)))
                    sensingTable(i,j) = k; % Record the finding
                    break; % Stop checking this angle for this agent
                end 
            end
        end
    end
end