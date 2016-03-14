% Philip Twu and Magnus Egerstedt
% Fall 2010
% © 2010 by Georgia Institute of Technology. All rights reserved.
%
% function [x,lostX] = recoverAgents(x,lostX,delta)
% @param x (N x 2) matrix of agent states, where row i gives the
%                  [x-position y-position] of agent i
% @param lostX (N x 2) matrix of agent states for the lost agents, where 
%              row i gives the [x-position y-position] of lost agent i
% @param delta The maximum distance in which an agent can sense anything
% @return x The updated matrix of agent states, where lost agents that are 
%           'found' have been concatenated to the bottom rows 
% @return lostX The updated matrix of lost agent states, where lost agents
%               that are found have had their rows removed
%
% Takes in information about agents and the lost agents.  A lost agent is
% found by a normal agent if it is within some dist to it.  This function
% figures out which lost agents are found and updates the state and lost
% agent state vectors accordingly, by moving all found agents to 'x'.

function [x,lostX] = recoverAgents(x,lostX,delta)

% Implicitly find the number of agents and lost agents
N = size(x,1);
lostN = size(lostX,1);

% The distance between an active agent an a lost agents center for them to
% find each other
findDist = delta/3;

if(lostN ~= 0) % Only check if there are still agents to be found
    % Check to see if any of the lost agents were found
    for i=lostN:-1:1 % Try each lost agent i
        for j=1:N % Find with each normal agent j
            if(norm(lostX(i,1:2) - x(j,1:2)) <= findDist)
                % If found, transfer them fron the lostX vector to the x vector
                x(end+1,:) = lostX(i,:);
                lostX(i,:) = [];
                break;
            end
        end
    end
end