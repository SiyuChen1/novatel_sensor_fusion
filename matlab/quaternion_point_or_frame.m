clear
clc

R1 = rotz(30) * rotx(45) * roty(60);
% the equivalent quaternion is
% [0.72331741 0.20056212 0.5319757  0.39190384]
% first value is real part q_w

% q1 is the right quaternion, use point!!!
q1 = quaternion(R1, 'rotmat', 'point');

% q1_inv is the inverse of the quaternion, use frame
q1_inv = quaternion(R1, 'rotmat', 'frame');

% difference between frame and point see
% https://de.mathworks.com/help/robotics/ref/quaternion.html
% Sensor Fusion and Tracking Toolbox defaults to frame rotation.

% https://de.mathworks.com/matlabcentral/answers/810105-question-regarding-quaternion-conventions-particularly-with-respect-to-point-vs-frame-rotations

q1_identity = q1_inv * q1;

% define a new rotation
R2 = roty(30) * rotz(45) * roty(30);
q2 = quaternion(R2, 'rotmat', 'point');
q2_inv = quaternion(R2, 'rotmat', 'frame');

% ground truth value
R_gt = R2 * R1;
q_gt = quaternion(R_gt, 'rotmat', 'point');
q_gt_inv = quaternion(R_gt, 'rotmat', 'frame');

% note that q and -q represent the same rotation
% so we check the both values, only one value should be true
% flag1 should be 1(true)
flag1 = xor(norm(q_gt - q2 * q1) < 1e-10, norm(q_gt + q2 * q1) < 1e-10);

% flag2 should be 1(true)
flag2 = xor(norm(quatinv(q_gt) - q_gt_inv) < 1e-10, norm(quatinv(q_gt) + q_gt_inv) < 1e-10);

% flag3 should be 1(true)
% if r = q * p, then r_inv = p_inv * q_inv
% https://personal.utdallas.edu/~sxb027100/dock/quaternion.html
% The inverse of a quaternion refers to the multiplicative inverse (or 1/q) and can be computed by q_inv = q'/(q*q')
% If a quaternion q has length 1, we say that q is a unit quaternion.
% The inverse of a unit quaternion is its conjugate, q_inv = q'
flag3 = xor(norm(q_gt_inv - q1_inv * q2_inv) < 1e-10, norm(q_gt_inv + q1_inv * q2_inv) < 1e-10);
