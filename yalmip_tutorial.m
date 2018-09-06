%
addpath('C:\Program Files\Mosek\8\toolbox\r2014a');
addpath(genpath('.\Examples\YALMIP-master'));

% test yalmip+mosek
%yalmiptest('mosek');

% variables
t_scalar = sdpvar; % scalar
t_vec = sdpvar(5,1); % vector
t_mat = sdpvar(5); %<=> sdpvar(5,5) % matrix (SYMMETRIC)
t_mat_rect = sdpvar(5,7); % full matrix (non-symmetric)
t_mat_sqr_full = sdpvar(5,5,'full'); % full SQUARE matrix (non-symmetric)

% yalmip varibles "behave" like usual variable:
% this defines FUCNTIONs of variables
t_scalar*rand(4);
[1 2 3 4 5]*t_vec;
t_mat*t_vec;
norm(t_vec);
det(t_mat);

% objective (simply a function of variables. SCALAR!)
%---------------
h = norm(t_vec);

% constraints
%---------------

% -- linear inequalities (=>LP):
c = [1 4 2 4 3]';
Constraint_linear1 = c'*t_vec <= 7;

A = randn(5);
b = randn(5,1);
Constraint_linear2 = A*t_vec<=b;

% -- second order cones (=>SOCP)
Constraint_SOC = norm(t_vec)<=5; % NOT efficient
% or equivalently
Constraint_SOC = cone(t_vec,5);

% or more complex expressions
Constraint_SOC = cone(A*t_vec,t_scalar);

% or many constraints at once
Constraint_SOC_many = cone(t_mat); % see help of cone

% -- positive semidefinite constraints (linear matrix inequalities) (=> SDP)
Constraint_LMI = t_mat>=0; % <=> t_mat is a positive semidefinite matrix (t_mat MUST be symmetric; otherwise, you'll get entrywise non-negativity)

% odd examples
t_vec*t_vec' >= t_mat %<== non-linear matrix inequality <--> (t_vec*t_vec'-t_mat) is PSD
t_mat >= t_vec*t_vec' %<== LINEAR matrix inequality <--> the matrix [t_mat t_vec; t_vec' 0] is PSD 

% -- concatenating constraints
Constraints_ALL = [Constraint_linear1; Constraint_linear2];
Constraints_ALL = Constraint_linear1 + Constraint_linear2; % same thing

%% solve!!
optimize(Constraints_ALL,h);

%% evaluate solution
double(t_vec)
double(h)