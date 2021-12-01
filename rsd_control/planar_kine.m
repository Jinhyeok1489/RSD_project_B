function T = planar_kine(l1, l2, th1, th2)
%   Derive forward kinematics of planar 2R manipulator
%   INPUT:
%           l1, l2: link length
%   OUTPUT: 
%           th1, th2: joint angles

TB1 = DH_modified(0, 0, 0, th1);
T12 = DH_modified(0, l1, 0, th2);
T2E = DH_modified(0, l2, 0, 0);

TB2 = TB1*T12;
TBE = TB2*T2E;

T = TBE;

end