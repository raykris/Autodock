clear; close; clc
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%==========Tanker Dynamics ====================%
% Data obtained from MSS toolbox, wamit file Tanker, by T. I. Fossen.
syms xn yn psi  % Position and orientation
syms u v r      % linear and angular velocity
syms X Y N      % Forces and moments   

eta     = [xn;yn;psi];      % Position and orientation (NED)
nu      = [u;v;r];          % Linear and angular velocity (NED)'
tau     = [X;Y;N];          % Forces and moments (BODY)
% Tanker specifications
load tanker
m       = vessel.main.m; 
T       = vessel.main.T;
B       = vessel.main.B; 
Lpp     = vessel.main.Lpp;
nabla   = vessel.main.nabla;
rho     = vessel.main.rho; 
CG      = vessel.main.CG;
R44     = vessel.main.k44;
R55     = vessel.main.k55;
R66     = vessel.main.k66;
A0      = vessel.A(:,:,1);      % Zero frequency added mass matrix
B0      = zeros(3);             % vessel.B(:,:,1); % Potential damping at omega = 0

% -------Hydrodynamic forces from------%
[MRB_CO, CRB_CO] = rbody(m,R44,R55,R66,[0;0;r],CG);    % Finding MRB and CRB in CO
       
MRB_CO  =   [MRB_CO(1,1),MRB_CO(1,2),MRB_CO(1,6);
             MRB_CO(2,1), MRB_CO(2,2), MRB_CO(2,6);
             MRB_CO(6,1), MRB_CO(6,2),MRB_CO(6,6)];          % 3-DOF 
        
CRB_CO  =   [CRB_CO(1,1),CRB_CO(1,2),CRB_CO(1,6);
             CRB_CO(2,1), CRB_CO(2,2), CRB_CO(2,6);
             CRB_CO(6,1), CRB_CO(6,2),CRB_CO(6,6)];             % 3-DOF 
   
M_A     =   [A0(1,1), 0, 0;
             0 A0(2,2), A0(2,6); 
             0, A0(6,2), A0(6,6)];           % Added mass matrix, 3-DOF
        
R       =   Rzyx(0,0,eta(3));               % Rotation matrix from Body to NED    

C_A     =   m2c(M_A,nu);
C       =   CRB_CO+C_A;
M       =   MRB_CO+M_A;
D       = zeros(3,3);
D(1,1)=M(1,1)/160; D(2,2)=M(2,2)/19; D(3,3)=M(3,3)/21; % The part of N_ITTC(A1) in D(1,1) is not included, this is a linearisation of the quadratic surge recistance and thus at lower speed has less impact. 
% ------- Creating dynamic functions------%
eta_dot = R*nu;
nu_dot  =M\(tau-C*nu-D*nu);

states = [eta;nu];
input = tau;
matlabFunction(eta_dot,nu_dot,'file','TankerDynamics','vars',{states,input},'Comments','in1=states=[xn;yn;psi;u;v;r], in2=inputs=[X;Y;N]') 



%==========Tanker Course auto pilot====================%
% Based on pole placement algorithms from Fossen 2021 Algorithm 15.1,
% and Ch.7.

syms wb zeta
K   = 1/D(3,3);   % 1/(-Nr)
T   = M(3,3)*K;   % (Iz-Nrdot)*K


wn  = 1/(sqrt(1-2*zeta^2+sqrt(4*zeta^4-4*zeta^2+2)))*wb;

Kp  = wn^2*T/K;
Kd  = (2*zeta*wn*T-1)/K;
Ki  = wn^3*T/(10*K);

matlabFunction(Kp,Kd,Ki,'file','CourseAutoP','vars',{wb,zeta}) 











%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%==========Tanker PID controller====================%
% This is from Fossen (2021) p.532
%  syms wb1 wb2 wb6 zeta1 zeta2 zeta6 
%  S = Smtrx([0;0;r]);
%  wb=[wb1; wb2; wb6];
%  zeta = [zeta1; zeta2; zeta6];
%  Omega_b = diag([wb1,wb2, wb6]);
%  Z = diag([zeta1,zeta2, zeta6]);
%  
%  wn1=1/((1-2*zeta1^2+(4*zeta1^4-4*zeta1^2+2)^(1/2))^(1/2))*wb1;
%  wn2=1/((1-2*zeta2^2+(4*zeta2^4-4*zeta2^2+2)^(1/2))^(1/2))*wb2;
%  wn6=1/((1-2*zeta6^2+(4*zeta6^4-4*zeta6^2+2)^(1/2))^(1/2))*wb6;
% Omega_n = diag([wn1,wn2,wn6]);
% 
% Kp = R*M*R'*(Omega_n'*Omega_n);
% Kd = 2* R*M*R'*Z*Omega_n-R*(C+D)*R';%+R*M*S*R;
% Ki = 1/10*Kp*Omega_n;
% matlabFunction(Kp,Kd,Ki,'file','TankerPID_Gains','vars',{wb,zeta,psi,nu},'Comments','in1=bandwidths=[wb1;wb2;wb6], in2=damping ratios=[zeta1;zeta2;zeta6], in3=yaw angle=psi, in4=states=[u,v,r]') % Because of C, u,v,r has to be computed. 



