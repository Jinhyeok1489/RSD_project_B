function T = DH_modified(alp, a, d, th)
%   Compute DH transformation matrix using DH parameters
%   INPUT:
%           alp:    angles between x axis
%           a:      distance between x axis
%           d:      distance between z axis
%           th:     angles between z axis
%   OUTPUT:
%           T:      transformation matrix

rotx = [1, 0, 0, 0; 0, cos(alp), -sin(alp), 0;  ...
        0, sin(alp), cos(alp), 0; 0, 0, 0, 1];
transa = [1, 0, 0, a; 0, 1, 0, 0; ...
        0, 0, 1, 0; 0, 0, 0, 1];
transz = [1, 0, 0, 0; 0, 1, 0, 0; ...
        0, 0, 1, d; 0, 0, 0, 1];
rotz = [cos(th), -sin(th), 0, 0; sin(th), cos(th), 0, 0;
        0, 0, 1, 0; 0, 0, 0, 1];

T = rotx*transa*transz*rotz;

end