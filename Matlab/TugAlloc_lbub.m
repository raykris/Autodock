function [df,da,s] = TugAlloc_lbub(tau,T, dT,f0,a0,fLim,aLim,fRate,aRate)
% tau   = Forces and moments vector
% T     = Thrust  coefficients matrix for all effectors  /actuators
% dT    =   dT/da,  only rotatable trhusters
% f0    = Initial force (N)
% a0    = Initial angle (rad)
% fLim  = Thrusters min and max values [min,max] (N)
% aLim  = Allowable sector for rotatable thrusters [min, max] (rad)
% fRate = Change in thrust force (N/step)
% aRate = Change in angle (rad/step)

ne      = length(f0);   % Number of effectors
na      = length(a0);   % Number of rotating thrusters   
nd      = length(tau);  % Number of DOF (forces and moments)

% Parameters for azimuth thrusters
f_min   = fLim(:,1);
f_max   = fLim(:,2);
df_min  = -fRate;
df_max  = fRate;

a_min   = aLim(:,1);
a_max   = aLim(:,2);
da_min  = -aRate;
da_max  = aRate;

lb =    [df_min;
         da_min;
         ones(nd,1)*-inf];

ub =    [df_max;
         da_max;
         ones(nd,1)*inf];

% Weighting matrices
 W       = eye(ne)*diag([5,80,80]);      % Weighting matrix for thrusters
W1=W; % W1 =  diag([3/4*abs(f0(1))^(-0.5),3/4*abs(f0(2))^(-0.5)])*20;
W2=W; % W2 =  diag([3/2*abs(f0(1))^(0.5),3/2*abs(f0(2))^(0.5)])*20;

Ohm     = eye(na)*1;        % Weighting matrix for change in azimuth angle
Q       = eye(nd)*diag([1e4,1e4,1e5]);     % Weight matrix for slack variables  

PHI =   [W1, zeros(ne,na), zeros(ne,nd);
         zeros(na,ne), Ohm, zeros(na,nd);
         zeros(nd,ne), zeros(nd,na), Q];  

P = [tau', f_min', f_max', a_min', a_max',a0', f0']';
R = zeros(size(PHI,1),size(P,1));
R(1:ne,end-(ne-1):end) = W2;


% Equalities
A1          = [T, dT, eye(nd)];
C1          =  zeros(nd,length(P));
C1(1:nd,1:nd) = eye(nd); 
C1(1:nd,end-(ne-1):end) = -T;

% Inequalities 

A2      =  [-eye(ne), zeros(ne,na), zeros(ne,nd);
             eye(ne), zeros(ne,na), zeros(ne,nd);
             zeros(na,ne),-eye(na), zeros(na,nd);
             zeros(na,ne), eye(na), zeros(na,nd)];

C2      = [zeros(ne,nd), -eye(ne), zeros(ne,ne),zeros(ne,na),zeros(ne,na),zeros(ne,na), eye(ne);
           zeros(ne,nd), zeros(ne,ne), eye(ne), zeros(ne,na),zeros(ne,na),zeros(ne,na),-eye(ne);
           zeros(na,nd), zeros(na,ne), zeros(na,ne), -eye(na), zeros(na,na),eye(na),zeros(na,ne);
           zeros(na,nd), zeros(na,ne), zeros(na,ne), zeros(na,na), eye(na), -eye(na),zeros(na,ne)];




options = optimoptions(	'quadprog',	'Algorithm','active-set');
x0 = [f0;a0;zeros(nd,1)];
X = quadprog(PHI,(R*P).',A2,C2*P,A1,C1*P,lb,ub,x0, options);
df = X(1:ne);
da = X(ne+1:ne+na);
s = X(ne+na+1:na+ne+nd);
end