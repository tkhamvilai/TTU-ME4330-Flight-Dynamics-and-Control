% Adaptive Control
% What if we want to use the linear controller
% but we don't know anything about the system
% i.e., we don't know the value of the A and B matrices
% We know that our system is in the form of
% x_dot = Ax + Bu but we don't know the value of A and B
% Spoiler: we keep changing the gains during the execution
% K will not be a constant number anymore