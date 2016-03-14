% Philip Twu and Magnus Egerstedt
% Fall 2010
% © 2010 by Georgia Institute of Technology. All rights reserved.
%
% function [R]=rotmatrix(theta)
% @param theta The angle (radians) in which the rotation matrix should
%              rotate a vector in the CCW direction 
% @return R The (2x2) rotation matrix such that Rv rotates v CCW by theta
%
% This function returns the (2x2) rotation matrix such that when multiplied
% to a vector, rotates it by 'theta' radians in the CCW direction

function [R]=rotmatrix(theta)

R = [cos(theta) -sin(theta);...
    sin(theta) cos(theta)];