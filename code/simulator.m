% Philip Twu and Magnus Egerstedt
% Fall 2010
% © 2010 by Georgia Institute of Technology. All rights reserved.
%
% function [missionResults]=simulator(controllerFile,varargin)
% @param controllerFile The file containing the implementations of the
%                       decentralized controllers and guards
%                       will be located in the file: 'controllerFile.m'
% @param varargin An optional argument containing the filename (minus
%                 extension) of the saved simulation state to be loaded
%                 For example, if simulation state is saved as 'mysave.mat'
%                 the argument will be 'mysave'.
% @return missionResults A (W x 2) matrix, where W is the number of
%                        waypoints, such that the ith row is of the form
%                        [wpCleared_i guardCleared_i], indicating whether
%                        the ith waypoint and ith guard were cleared when
%                        the simulation ended
%
% This is the main function which simulates the multi-agent system as it
% crosses the obstacle course, gets control signals from each agent, gives
% simulated sensor data to each agent, and keeps track of waypoint progress
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
% We will try to use the storage axis as much as possible for everything.
% Therefore, given some pt in storage (X,Y) to plot, all calls to plot() or
% drawCircle() or text() needs to pass in X and Y in the reverse order,
% i.e., plot(Y,X)

function [missionResults]=simulator(controllerFile,varargin)

close all

%% Simulation Settings
ACTUATOR_SATURATION_ON = 1; % Set to 1 to turn on actuator saturation,
% agent speeds are bounded by maximum of uMax,
% which is defined below
AGENT_COLLISION_ON = 1; % Set to 1 to turn on agent-to-agent collisions
OBSTACLE_COLLISION_ON = 1; % Set to 1 to turn on agent-obstacle collisions
SENSOR_NOISE_ON = 1; % Set to 1 to turn on sensor noise
LOCAL_COORD_FRAME_ON = 1; % Set to 1 for each agent to have its own random
  % local coordinate frame rotation, as in each agent has its own belief 
  % of where +x and +y axes are.  Set to 0 for all to agree with truth.
LOCAL_BEACON_PERMUTE_ON = 1; % Set to 1 for each agent to have its own 
  % random (but fixed) locally permuted indexing of beacon numbers for
  % clearing search and formation waypoints.  What this means is that 
  % beacon 1 may be known as beacon 2 to agent 1, and beacon 5 to agent 2.

RECORD_MOVIE = 0; % Set to 1 to record movie if mission is accomplished

%% Random Number Generation Settings
% Note that two random number generators will be used, one for
% mission-level setup such as axis orientations, etc.  The second will be
% used purely for generating sensor noise.  That way, whether caller turns
% sensor noise on or off, the same mission-level randomness will be
% experienced

% Set up random seeds
missionRandSeed = 39586;
sensorRandSeed = 837395;

% Set the random number stream used for sensor noise and 
missionRandStream = RandStream('mt19937ar','Seed',missionRandSeed);
sensorRandStream = RandStream('mt19937ar','Seed',sensorRandSeed);

%% Agent Settings (Coords are given s.t. x axis pts down, y axis pts right,
%% i.e., coord frame used for storage of image from imread(), not imagesc()
% Initial agent states (First agent is the leader)
x = [130 110; 100 400; 400 150; 250 300; 220 180; 123 263];

% Positions of the "lost" agents
lostX = [1199,1356; 1022,1543; 1022,1790; 1199,1946; 1400,1543; 1400,1790];

% The radius of an agent.  Used for collisions between agents, etc.
agentRadius = 15;

% Implicity determine the number of agents from the state vector
N = size(x,1);
initialN = N; % The number of agents that are initially in the network
lostN = size(lostX,1); % The number of lost agents that need to be found
grandTotalN = N + lostN;

% Agent local reference frames (randomly assigned), where agent i's frame
% is rotated CCW by theta(i)
if(LOCAL_COORD_FRAME_ON)
    theta = rand(missionRandStream,1,grandTotalN)*2*pi;
else
    theta = zeros(1,grandTotalN); % Local rotation of 0 for all means global frame
end

% The fastest that an agent is able to move (actuator saturation)
uMax = 1000;

% Each agent can store up to 10 pieces of information in storage
agentMemorySize = 10;
saveData = zeros(grandTotalN,agentMemorySize);

%% Sensing Settings
% Angles to sense for obstacles
numPhiAngles = 16;
phi = (0:numPhiAngles-1)/numPhiAngles*2*pi;

% The maximum dist. in which two agents can have information flow
delta = 200;

% Sensor noise 1-sigma value
sensorSigma = 1;

% Implement noise
if(~SENSOR_NOISE_ON)
   sensorSigma = 0; % 1-sigma noise value of 0 = no noise
end

%% Waypoint Settings
wp = [300 600;... % Row i corresponds to the x and y pos of waypoint i.
    91 1936;...
    1461 1311;...
    1203 1669;...
    1160 530;
    1232 247];

% Waypoint names (change ordering of the numbers to change order of Waypts)
WP_GOTO = 1;
WP_TUNNEL = 2;
WP_SMALLROCKS = 3;
WP_SEARCH = 4;
WP_SPLITMERGE = 5;
WP_FORMATION = 6;

wpCount = size(wp,1); % Implicitly find the number of waypoints

currTargetWP = WP_GOTO; % Start with needing to clear the first waypoint

wpRadius = 40; % The distance an agent needs to get within a waypoint in
% order to clear it

firstCall = ones(grandTotalN,1); % Used to determine whether it is the
% first time a controller is used

%% Allocate array to hold the mission results
missionResults = zeros(wpCount,2); % Each row will say if that wp was
% cleared, and if the guard was
% cleared

%% Beacon Settings for clearing WP 4
% Beacon locations
beacons = [924 1311; 1466 1313; 1468 1994; 924 1995];

% Precompute local beacon permutations
numBeacons = size(beacons,1);
beaconPermIdx = [];
if(LOCAL_BEACON_PERMUTE_ON)
    % Randomly permute indices for each agent
    for i=1:grandTotalN
        beaconPermIdx(i,:) = randperm(missionRandStream,numBeacons);
    end
else
    % Leave each index unpermuted
    for i=1:grandTotalN
        beaconPermIdx(i,:) = 1:numBeacons;
    end
end

%% Formation Settings for clearing WP 6
% Beacon locations
formationCoords = [1081 163; 1081 330; 1158 392; 1312 392; 1379 330; 1379 163; 1312 105; 1158 105; 1186 248; 1229 207; 1229 287; 1272 248];

% Precompute localbeacon permutations
numFormationCoords = size(formationCoords,1);
formationPermIdx = [];
if(LOCAL_BEACON_PERMUTE_ON)
    % Randomly permute indices for each agent
    for i=1:grandTotalN
        formationPermIdx(i,:) = randperm(missionRandStream,numFormationCoords);
    end
else
    % Leave each index unpermuted
    for i=1:grandTotalN
        formationPermIdx(i,:) = 1:numFormationCoords;
    end
end

%% Integrator Settings
dt = 0.01; % Time step for Euler integration & simulation

%% Visualization Settings
pauseTime = 0.005;
frameCount = 0;

%% No collisions have occured initially (defualt values)
agentCol = 0;
obstacleCol = 0;

%% Load the simulation state if there is one
if(~isempty(varargin))
    % Get save file name
    savedSimulationName = varargin{1};
    
    % Load save file
    eval(['load ' savedSimulationName])
    disp(['Simulation state "' savedSimulationName '.mat" Loaded'])
    
    % Re-initialize random number generator for sensor noise
    sensorRandStream.State = sensorRandStreamState;
end

%% Make an initial backup of the simulation state (must come after simulation state is loaded, if one is found)
save_currTargetWP = currTargetWP;
save_x = x;
save_lostX = lostX;
save_missionResults = missionResults;
save_saveData = saveData;
save_firstCall = firstCall;
save_N = N;
save_sensorRandStreamState = sensorRandStream.State;

%% Get the user specified controllers and guards
[controllers,guards] = controllerFile();

%% Show the initial topology and pause
A = determineTopology(x,delta);
visualize(x,lostX,A,wp,currTargetWP,wpRadius,agentRadius);

if(RECORD_MOVIE)
    framecount = 1;
    set(gca,'nextplot','replacechildren');
    moviebuffer(framecount) = getframe(gcf);
end

disp('Simulation Ready, press any key to continue.')
pause
disp('Simulation Started!')

while(currTargetWP <= wpCount && ~agentCol && ~obstacleCol)
    %% Check for agent recovery
    if(currTargetWP == WP_SEARCH)
        [x,lostX] = recoverAgents(x,lostX,delta);
        % Recompute N based on new state data.
        N = size(x,1);
    end
    
    %% Determine the information flow network
    A = determineTopology(x,delta);
    
    %% Check for collisions with obstacles
    if(AGENT_COLLISION_ON)
        [agentCol] = checkAgentCollisions(x,agentRadius);
    end
    
    %% Plot the agents
    visualize(x,lostX,A,wp,currTargetWP,wpRadius,agentRadius);
    % Pause between plot refreshes
    pause(pauseTime);
    frameCount = frameCount + 1;
    
    %% Have the agents sense for obstacles within (delta) distance
    [sensingTable]=senseObstacles(x,theta,phi,delta,currTargetWP);

    % Check for obstacle collisions
    if(OBSTACLE_COLLISION_ON)
        % Check to see if an agent-obstacle collision has occured using
        % the un-noisy sensor measurements
        obstacleCol = sum(sum(sensingTable <= agentRadius)) > 0;
    end
    
    % Add Agent-to-Obstacle Sensing Noise
    sensingTable = sensingTable + sensorSigma*randn(sensorRandStream,size(sensingTable));
    
    %% Continue only if no collision has occured
    if(~agentCol && ~obstacleCol)
        u = zeros(N,2); % Matrix used to store control signals of each agent
        for i=1:N % Calculating the control signal for agent i
            %% Simulate sensing between agents
            nbrData = [];
            % Find the relative displacements between neighbors and agent i
            for j=1:N
                if(A(i,j) == 1)
                    % Agent i senses relative displacement (in own coord frame)
                    % and the identifier of its neighbors
                    nbrData = [nbrData; (rotmatrix(-theta(i))*(x(j,:) - x(i,:))')' j];
                end
            end

            % Add Agent-to-Agent Sensing Noise
            if(~isempty(nbrData))
                nbrData(:,1:2) = nbrData(:,1:2) + sensorSigma*randn(sensorRandStream,size(nbrData(:,1:2)));
            end
            
            %% Do actions specific for the leader agent
            wpData = [];
            if(i == 1) % Only the leader is allowed to know the waypoint data
                % Have the leader know the waypoint data
                wpData = rotmatrix(-theta(i))*(wp(currTargetWP,:) - x(i,:))';
            end
            
            % Add Leader-to-Waypoint Sensing Noise
            wpData = wpData + sensorSigma*randn(sensorRandStream,size(wpData));
            
            %% Invoke the decentralized controllers using sensor data
            switch currTargetWP
                case WP_GOTO
                    [tempU, tempSaveData] = controllers{WP_GOTO}(i,nbrData,wpData,sensingTable(i,:)',[],saveData(i,:)',delta,agentRadius,firstCall(i));
                case WP_TUNNEL
                    [tempU, tempSaveData] = controllers{WP_TUNNEL}(i,nbrData,wpData,sensingTable(i,:)',[],saveData(i,:)',delta,agentRadius,firstCall(i));
                case WP_SMALLROCKS
                    [tempU, tempSaveData] = controllers{WP_SMALLROCKS}(i,nbrData,wpData,sensingTable(i,:)',[],saveData(i,:)',delta,agentRadius,firstCall(i));
                case WP_SEARCH
                    % Compute the relative displacements between each agent
                    % and the beacon locations
                    relativeBeacons = [];
                    for m=1:size(beacons,1)
                        relativeBeacons(m,:) = (rotmatrix(-theta(i))*(beacons(m,:) - x(i,:))')';
                    end
                    % Permute beacon order based on pre-defined local agent
                    % permutation
                    relativeBeacons = relativeBeacons(beaconPermIdx(i,:),:);
                    % Add Agent-to-Beacon sensing noise
                    relativeBeacons = relativeBeacons + sensorSigma*randn(sensorRandStream,size(relativeBeacons));
                    % Call controller
                    [tempU, tempSaveData] = controllers{WP_SEARCH}(i,nbrData,wpData,sensingTable(i,:)',relativeBeacons,saveData(i,:)',delta,agentRadius,firstCall(i));
                case WP_SPLITMERGE
                    [tempU, tempSaveData] = controllers{WP_SPLITMERGE}(i,nbrData,wpData,sensingTable(i,:)',[],saveData(i,:)',delta,agentRadius,firstCall(i));
                case WP_FORMATION
                    % Compute the relative displacements between agents and
                    % the formation spots
                    relativeFormations = [];
                    for m=1:size(formationCoords,1)
                        relativeFormations(m,:) = (rotmatrix(-theta(i))*(formationCoords(m,:) - x(i,:))')';
                    end
                    % Permute formation spot order based on pre-defined
                    % local agent permutation
                    relativeFormations = relativeFormations(formationPermIdx(i,:),:);
                    % Add Agent-to-Formation Spot Sensing Noise
                    relativeFormations = relativeFormations + sensorSigma*randn(sensorRandStream,size(relativeFormations));
                    % Call controller
                    [tempU, tempSaveData] = controllers{WP_FORMATION}(i,nbrData,wpData,sensingTable(i,:)',relativeFormations,saveData(i,:)',delta,agentRadius,firstCall(i));
                otherwise
                    tempU = [0 0];
                    tempSaveData = [];
            end
            
            % Rotate back the control signal
            u(i,:) = (rotmatrix(theta(i))*tempU)';
            
            %% Apply scaling for actuator saturation
            if(ACTUATOR_SATURATION_ON)
                if(norm(u(i,:)) > uMax)
                    u(i,:) = u(i,:) ./ norm(u(i,:)) * uMax;
                end
            end
            
            %% Resize (concatenate if needed) the save data and store it
            if(isempty(tempSaveData))
                % If controller returned empty matrix to save, make it a
                % zero vector instead
                tempSaveData = zeros(agentMemorySize,1);
            else
                % Only take the first column, delete the rest
                tempSaveData = tempSaveData(:,1);
                % Pad or concat depending on the length
                tempSaveDataLength = length(tempSaveData);
                if(tempSaveDataLength > agentMemorySize)
                    % Too much data, throw the extra away
                    tempSaveData = tempSaveData(1:agentMemorySize);
                else
                    % Too little data, pad with zeros
                    tempSaveData = [tempSaveData;zeros(agentMemorySize-tempSaveDataLength,1)];
                end
            end
            saveData(i,:) = tempSaveData';
            
            %% Controller for this agent has finished
            firstCall(i) = 0;
            
            %% Have the leader check if the user-def mode guard is cleared
            if(i == 1)
                guardCleared = guards{currTargetWP}(nbrData,wpData,sensingTable(i,:)',saveData(i,:)',delta,agentRadius);
            end
        end
        
        %% Simulate the system one time step forward using the controls
        x = x + dt*u;
        
        %% Check to see if the current target waypoint is completed and
        %% update the waypoint if necessary
        % First check to see if the correct waypoint is reached by the
        % leader, with the right number of agents.
        % Then check waypoint-specific conditions.
        switch(currTargetWP)
            case WP_GOTO
                wpCleared = checkClearWaypoint(x,wp,WP_GOTO,wpRadius,delta,initialN);
            case WP_TUNNEL
                wpCleared = checkClearWaypoint(x,wp,WP_TUNNEL,wpRadius,delta,initialN);
            case WP_SMALLROCKS
                wpCleared = checkClearWaypoint(x,wp,WP_SMALLROCKS,wpRadius,delta,initialN);
            case WP_SEARCH
                wpCleared = checkClearWaypoint(x,wp,WP_SEARCH,wpRadius,delta,initialN+lostN);
            case WP_SPLITMERGE
                wpCleared = checkClearWaypoint(x,wp,WP_SPLITMERGE,wpRadius,delta,initialN+lostN);
            case WP_FORMATION
                wpCleared = checkFormation(x,formationCoords,agentRadius);
        end
        
        % Increment the current target WP AND user-defined guard condition
        % (as evaluated by the leader) if both are cleared
        if(wpCleared)
            % If this is the first time clearing the waypoint
            if(missionResults(currTargetWP,1) == 0)
                % Record that the waypoint was cleared
                missionResults(currTargetWP,1) = 1;
                % Output message to the terminal
                disp(['Waypoint ' num2str(currTargetWP) ' cleared, waiting for guard' num2str(currTargetWP) '() to clear...']);
            end
            
            if(guardCleared)
                % Record that the guard was cleared
                missionResults(currTargetWP,2) = 1;
                % Display message to the terminal
                disp(['guard' num2str(currTargetWP) '() cleared!']);
                % Update the current waypoint
                currTargetWP = currTargetWP + 1;
                % Re-initialize the firstCall flags for every agent
                firstCall = ones(initialN + lostN,1);
                
                % Make a backup of the simulation state
                save_currTargetWP = currTargetWP;
                save_x = x;
                save_lostX = lostX;
                save_missionResults = missionResults;
                save_saveData = saveData;
                save_firstCall = firstCall;
                save_N = N;
                save_sensorRandStreamState = sensorRandStream.State;
            end
        end
    end

    % Save the movie frame
    if(RECORD_MOVIE)
        framecount = framecount + 1;
        moviebuffer(framecount) = getframe(gcf);
    end
    
end

% Add 50 frames of stillness after mission is completed for movie
if(RECORD_MOVIE)
    for i=1:50
        framecount = framecount + 1;
        moviebuffer(framecount) = getframe(gcf);
    end
end

%% Display the result of the mission
if(~agentCol && ~obstacleCol)
    % Mission Accomplished
    disp('Mission Accomplished! Congratulations!');
    set(gca,'Color','g')

    % Record the movie only if mission was accomplished
    if(RECORD_MOVIE)
        movie2avi(moviebuffer, 'finalproject.avi','compression','None');
    end
else
    % Mission Failed
    % First Display the correct message
    if(agentCol)
        disp('Mission Failed: Agent-to-Agent Collision!');
    elseif(obstacleCol)
        disp('Mission Failed: Agent-to-Obstacle Collision!');
    end
    set(gcf,'Color','r')
    
    % Ask the user if they would like to save waypoint data
    saveDecision = '';
    while(~strcmpi(saveDecision,'y') && ~strcmpi(saveDecision,'n'))
        saveDecision = input('Would you like to save the simulation state to re-start from the previous waypoint [Y/N]? ','s');
    end
    
    if(strcmpi(saveDecision,'y'))
        % Ask them what filename they wish to save it as
        save_name = '';
        while(length(save_name) == 0)
            save_name = input('Enter filename to save as (without extension): ','s');
        end
        
        % Restore the simulation state to the previous waypoint
        currTargetWP = save_currTargetWP;
        x = save_x;
        lostX = save_lostX;
        missionResults = save_missionResults;
        saveData = save_saveData;
        firstCall = save_firstCall;
        N = save_N;
        sensorRandStreamState = save_sensorRandStreamState;
        
        % Save the simulation state variables
        save(save_name,'currTargetWP','x','lostX','missionResults','saveData','firstCall','N','sensorRandStreamState');
    end
end