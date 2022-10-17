
% 500 is an inappropriate size on a laptop for omega :]

NOMINAL_DK = wolp_ic;
NMEAS_DK = 1;
NCONT_DK = 1;
BLK_DK = [-1 1; -1 1; -1 1; -1 1; -1 1; -1 1; 1 2]; % block structure
OMEGA_DK = logspace(-2,4,100);  
% autoinfo = [initial iteration...
%             final iteration; 
%             visualisation flag: 1 show results or 2 dont. Try not to :)
%             size of blocks and max size of dynamic scaling? huh?
AUTOINFO_DK = [1 4 1 4*ones(1,size(BLK_DK,1))];
NAME_DK = 'mds';


% Iteration Summary                                          
% -----------------------------------------------------------
% Iteration #                 1         2         3         4
% Controller Order            4         8         4         4
% Total D-Scale Order         0         4         0         0
% Gamma Acheived         16.973    13.823    13.715    13.799
% Peak mu-Value          14.160    13.806    13.693    13.768
%  
%  Next MU iteration number:  5