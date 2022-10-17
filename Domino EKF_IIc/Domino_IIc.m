function [X, P] = Domino_I(dt, X, P, W, wvar, A, avar, gps_heading, ghvar, gps_velocity, M, mvar )
%DOMINOI EKF Attitude Algorithm
%   This forms a "standard" building block EKF on which other features can
%   be added and experimented with. Uses magnetometer for heading at low
%   speed and gps for heading at higher speeds. 
% 
% Inputs
% dt: Time period [seconds]
% X: State vector. [q0, q1, q2, q3, bx, by, bz]
% P: Covariance
% W: Gyro angular rates [radians per second]
% wvar: gyro variance
% A: Accelerometer [x, y, z] (G)
% avar: accelerometer variance
% gps_heading: 
% ghvar: gps heading variance
% gps_velocity:
% M: Magnetometer [x, y, z]
% mvar: magnetometer variance
%
% Outputs
% X: State vector
% P: Covariance
% 
% Other notes
% - Upper case is vector/matrix, lower case is scalar 
% - In most cases is would be preferable to use additional kalman filters
% than to extend the state vector due to the computational cost of
% inverting a large matrix!
% - No pitch/roll correction for heading
%
% References
% [1] Xiaoying Kong. Inertial Navigation System Algorithms for Low Cost
% IMU. University of Sydney. Aug 27 2000. Dphil.
%
% Version History
%
% (c) Thomas Pile 2012

g = 9.80665;

% Initialise 
if isempty(X)
    X = [1, 0, 0, 0,  0, 0, 0]';
    P = zeros(length(X), length(X));
end

%%
% Setup variance 
% Predict variance
Q = eye(7)*wvar;
% Update variance
R = [avar,  0  , 0,      0   ;
      0  , avar, 0,      0   ;
      0  ,  0  , avar,   0   ;
      0  ,  0  ,  0,   ghvar];

%%
% Predict

X = X';

% This is a matrix representation of the Euler angles so they can be
% multiplied with the state Quanternion. We use this to Integrate the 
% quaternion, and so being angular rates this represents the first
% derivative of the position.
% [1]
ED = [0   , -W(1), -W(2), -W(3);
           W(1),   0  ,  W(3), -W(2);
           W(2), -W(3),  0   , -W(1);
           W(3),  W(2), -W(1),  0  ];
ED = 0.5 * ED;
% This is a matrix representation of the quaternion so the euler angles can
% be multiplied with it.
QE = [-X(2), -X(3), -X(4);
            X(1), -X(4),  X(3);
            X(4),  X(1), -X(2);
            -X(3), X(2),  X(1)];
QE = -0.5 * QE;
       
% State predict Jacobian matrix
F = [[ED,  QE]   ;
     zeros(3,7)]; % zeros to pad to 7x7

F;
X;
 
% State predict equations     
X = (eye(7) + F*dt) * X; 
P = F*P*F' + Q;

% normalise state vector
qmag = sqrt( X(1)*X(1) + X(2)*X(2) + X(3)*X(3) + X(4)*X(4) );
X(1) = X(1) / qmag;
X(2) = X(2) / qmag;
X(3) = X(3) / qmag;
X(4) = X(4) / qmag;

%%
% Correct

% State relation vector
h = [2*g*(X(2)*X(4)-X(1)*X(3));
     2*g*(X(3)*X(4)+X(1)*X(2));
     g*(X(1)^2-X(2)^2-X(3)^2-X(4)^2);
     atan2(2*(X(1)*X(4)+X(2)*X(3)), 1 - 2*(X(3)^2+X(4)^2))];
 
% Measurement Jacobian Matrix
% This is just the QE matrix above transposed. It represents the state 
% quaternion in a form that allows multiplication with the euler angles, so
% the measurement can be subtracted from the state to determine the
% residual.
H = 2*g* [-X(3),  X(4), -X(1),  X(2), 0, 0, 0;
           X(2),  X(1),  X(4),  X(3), 0, 0, 0;
           X(1), -X(2), -X(3),  X(4), 0, 0, 0;
            0  ,   0  ,   0  ,   0  , 0, 0, 0];

% Measurement
% Determine the heading source. GPS is moving, mag otherwise
% Get heading from mag (needs to be turned off if high roll/pitch)
heading = atan2(M(2), M(1));
% GPS heading
if (gps_velocity > 5)
    heading = gps_heading;
end
% This is just the values from measurement sensors (accel etc)
Z = [A, heading]';

% State Update equations
Residual = h-Z;
S = H*P*H' + R; 
K = P*H'/S;
X = X + K*(Residual);
P = (eye(7) - K*H)*P;

% normalise state vector
qmag = sqrt( X(1)*X(1) + X(2)*X(2) + X(3)*X(3) + X(4)*X(4) );
X(1) = X(1) / qmag;
X(2) = X(2) / qmag;
X(3) = X(3) / qmag;
X(4) = X(4) / qmag;


P;
X = X';

end

