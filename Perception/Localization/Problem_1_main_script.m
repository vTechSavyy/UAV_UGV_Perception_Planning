%% Model Based Estimation : Assignment # 7 - Problem # 1: 

% Title: Simulation test of the c2dnonlinear function: 

% Objective: Given a continous time non-linear differential system find its
% equivalent discrete time non-linear model. Then linearize the discrete
% time non-linear model so as to be able to apply Kalman Filter equations: 

%% Step 1: Setup simulation parameters: 

%  Generate a random initial condition: 

% Initial condition given for final exam: 
xk = [-0.40; 0.85; -0.60; -1.65];

% Test system has a 3x1 process noise vector: 

% Process noise for final exam - Fall 2016:
vk = [-0.77; 1.30; 1.65];

% Input vector: 
uk= zeros(2,1);

% Start time for integration: 
tk = 3;  % sec

% End time of numerical integration: 
tkp1 = 6; % sec


% Number of Runge Kutta steps: 
nRK = [60,120];

% Compute partial derivatives: 
idervflag=1;

% Name of the script for evaluating the continous time differential
% equation: 
fscriptname = 'fscript_ts01';

for ii = 1:1:2

%% Step 2: Perform Runge Kutta Numerical integration:
[fprinted,dfprinted_dxk,dfprinted_dvk] = ...
             c2dnonlinear_verified(xk,uk,vk,tk,tkp1,nRK(ii),fscriptname,idervflag);

%% Step 3: Compute bench mark values: 

A = ...
    [-0.43256481152822, -1.14647135068146,  0.32729236140865, ...
    -0.58831654301419;...
    -1.66558437823810,  1.19091546564300,  0.17463914282092, ...
    2.18318581819710;...
    0.12533230647483,  1.18916420165210, -0.18670857768144, ...
    -0.13639588308660;...
    0.28767642035855, -0.03763327659332,  0.72579054829330, ...
    0.11393131352081];
D = ...
    [ 1.06676821135919,  0.29441081639264, -0.69177570170229;...
    0.05928146052361, -1.33618185793780,  0.85799667282826;...
    -0.09564840548367,  0.71432455181895,  1.25400142160253;...
    -0.83234946365002,  1.62356206444627, -1.59372957644748];


% Calculate true fprinted: 
sysmodel_ct = ss(A,D,eye(4),zeros(4,3));
sysmodel_dt = c2d(sysmodel_ct,(tkp1-tk),'zoh');
[dfprinted_dxk_t,dfprinted_dvk_t] = ssdata(sysmodel_dt);
fprinted_true = dfprinted_dxk_t*xk + dfprinted_dvk_t*vk;


%% Step 4: Evaluate the error: 

error_vec= fprinted_true - fprinted; 

error_norm = norm(error_vec);


disp([' The error in numerical integration  for ', num2str(nRK(ii)) ,'  steps is: ']);
disp(error_norm);


% Diplay the fprinted value:

disp([' The fprinted value for ', num2str(nRK(ii)) ,' steps is']);

% fprinted
disp(fprinted);


disp([' The derivative of f w.r.t x for ', num2str(nRK(ii)) ,' steps is']);

% f w.r.t x: 
disp(dfprinted_dxk);


disp([' The derivative of f w.r.t v for ', num2str(nRK(ii)) ,' steps is']);

% f w.r.t x: 
disp(dfprinted_dvk);
end



