% Philip Twu and Magnus Egerstedt
% Fall 2010
% © 2010 by Georgia Institute of Technology. All rights reserved.
%
% function visualize(x,lostX,A,wp,currWP,wpRadius,agentRadius)
% @param x (N x 2) matrix of agent states, where row i gives the
%                  [x-position y-position] of agent i
% @param lostX (Z x 2) matrix of agent states for the lost agents, where 
%                  row i gives the [x-position y-position] of agent i
% @param A (N x N) adj matrix where A(i,j) = 1 iff info flows between i & j
% @param wp (W x 2) matrix of waypoint locations, where row i gives the 
%                  [x-position y-position] of wp i
% @param currWP The number of the current waypoint that needs to be cleared
% @param wpRadius The distance to get within a waypoint in order to count
%                 as "clearing" it.  A circle of radius "wpRadius" will be 
%                 drawn around each waypoint.
% @param agentRadius The radius of an agent, where agents are drawn as
%                    circles
%
% Takes in information about the agents and the environment.  Draws the
% background picture, agents, sensing links, obstacles, and scales the plot
% window accordingly for visualization.
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

function visualize(x,lostX,A,wp,currWP,wpRadius,agentRadius)

%% Settings
% Plot window settings
dynamicWindow = 1; % Whether or not window will zoom in on agents
agentPictureRatio = 0.2; % Plot window will be a square that is scaled
                          % such that the network takes up approx
                          % "agentPictureRatio" of the total screen area
minWindowWidth = 600; % The smallest width the window can shrink to
                          
% Background image settings
persistent backgroundIm backgroundRead
backgroundFilename = 'terrain_labeled.jpg';

% Read the background image if not already done so
if(isempty(backgroundRead))
    backgroundIm = imread(backgroundFilename);
    backgroundRead = 1;
end

% Agent drawing settings
leaderColor = 'r';
followerColor = 'b';
lostColor = 'k';

% Information link drawing settings
edgeColor = 'g';
edgeWidth = 2;

% Waypoint drawing settings
wpUnclearedColor = 'y';
wpClearedColor = 'b';

%% Initialization
% The dimensions of the image, don't let the plot window go past these
% Recall: worldDimensions(1) is height of image, (2) is width of image
worldDimensions = [size(backgroundIm,1),size(backgroundIm,2)];

% Deduce the number of agents and waypoints in the system
N = size(x,1);
W = size(wp,1);

%% Draw the environment
cla;
imagesc(backgroundIm);
hold on;

%% Draw the waypoints and link from leader to current waypoint
for i=1:W
    if(i < currWP)
        wpColor = wpClearedColor;
    else
        wpColor = wpUnclearedColor;
    end
    drawCircle([wp(i,2),wp(i,1)],wpRadius,[wpColor '-'],2);
    hold on
    h = text(wp(i,2),wp(i,1),num2str(i));
    set(h,'Color',wpColor,'FontWeight','bold');
end

%% Draw the edges
% Between adjacent agents
for i=1:N-1
    for j=i+1:N
        if(A(i,j))
            plot([x(i,2) x(j,2)],[x(i,1) x(j,1)],edgeColor,'LineWidth',edgeWidth);
            hold on
        end
    end
end

% Between leader and current wp
plot([x(1,2) wp(currWP,2)],[x(1,1) wp(currWP,1)],[wpUnclearedColor ':'],'LineWidth',edgeWidth);


%% Draw the agents (leader is drawn differently from followers)
drawCircle([x(1,2) x(1,1)],agentRadius,leaderColor,3);
h = text(x(1,2),x(1,1),'1');
set(h,'Color','w','FontWeight','bold');
for i=2:N
    drawCircle([x(i,2) x(i,1)],agentRadius,followerColor,3);
    h = text(x(i,2),x(i,1),num2str(i));
    set(h,'Color','w','FontWeight','bold');
end

%% Draw the lost agents
lostN = size(lostX,1);
for i=1:lostN
    drawCircle([lostX(i,2) lostX(i,1)],agentRadius,lostColor,3);
    h = text(lostX(i,2),lostX(i,1),'?');
    set(h,'Color','w','FontWeight','bold');
end

%% Determine the window axis dynamically to follow the agents
axis equal
if(dynamicWindow)
    % Find the vertical spread of the agents (within storage)
    minX = min(x(:,1));
    maxX = max(x(:,1));
    centerX = (minX + maxX)/2;
    % Find the horizontal spread of the agents (within storage)
    minY = min(x(:,2));
    maxY = max(x(:,2));
    centerY = (minY + maxY)/2;
    % Determine the appropriate window size to display the agents
    networkWidth = max([maxX - minX, maxY - minY]);
    windowWidth = max([minWindowWidth,networkWidth/sqrt(agentPictureRatio)]);
    windowWidth = min([windowWidth,worldDimensions(1),worldDimensions(2)]);
    
    % Adjust the center of the image
    if(centerY - windowWidth/2 < 1)
        centerY = 1 + windowWidth/2;
    elseif(centerY + windowWidth/2 > worldDimensions(2)-1)
        centerY = worldDimensions(2)-1-windowWidth/2;
    end
    if(centerX - windowWidth/2 < 1)
        centerX = 1 + windowWidth/2;
    elseif(centerX + windowWidth/2 > worldDimensions(1)-1)
        centerX = worldDimensions(1)-1-windowWidth/2;
    end
    
    % Set the axis
    axis([max(1,centerY - windowWidth/2), min(centerY + windowWidth/2,worldDimensions(2)-1),...
        max(1,centerX - windowWidth/2), min(centerX + windowWidth/2,worldDimensions(1)-1)]);
else
    axis([0 worldDimensions(2) 0 worldDimensions(1)])
end

% Set the color of the plot window, and turn off axes markings/labels
set(gcf,'Color','k')
set(gca,'Visible','off')