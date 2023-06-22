

clear; close; clc
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%==========Tugboat Dynamics CALC====================%
syms xn yn psi  % Position and orientation
syms u v r      % linear and angular velocity
syms X Y N      % Forces and moments   
eta     = [xn;yn;psi];      % Position and orientation (NED)
nu      = [u;v;r];          % Linear and angular velocity (NED)'
tau     = [X;Y;N];          % Forces and moments (BODY)

% measured values from BB bulldog drawing. 
Pos     = [0.5,0.6, 5.1, 13.2, 16.7, 20.9, 29.9, 31.1, 32.3,32.9, 34]-17.5; % Position
Draft   = [0,0.4, 1.2, 3.7, 4.4, 4.3, 4.1, 3.9, 3.1, 2.3, 0];               % Measured draf
PosKjol     = [6.7, 7.2, 7.8, 8.3, 12.6, 16.7]-17.5;                        % Position of "slingrekjøl"
DraftKjol   = [0, 1.1, 1.9, 2.7, 1.2, 0];                                   % Measured draf of "slingrekjøl" 

xq=-17:0.5:16.5;                         % Position of endpoints (under the waterline) with respect to the CO
vq = interp1(Pos,Draft,xq);             
vqK = interp1(PosKjol,DraftKjol,xq);
vqK(isnan(vqK))=0;                     

% Calculating the side area of the hull under the waterline.
Lsection = diff(xq);
n = length(Lsection);
A = zeros(n,1);
for i=1:n  
    A(i) =(vq(i)+vq(i+1))/2*Lsection(i);
end
B = 11.5; % beam / breadth / width
Volume  = sum(A)*B*0.7854*0.9;      % Computed displacement, based on "measured" draft, max beam and the differnce ellips=0.7853*rectange, and reduced of 10 percent. 
rho     = 1025;                     % Density of water
m       = Volume*rho*0.90;          % Mass from computed displacement. Reduced mass with 10% 
T       = max(vq);                  % Max measured draft from drawing. 
L       = 33.5;                     % Assumed Lpp 
CG      = [0,0,0];                  % Distance from CO, setting CG=CO 
R44     = 0;                        % Not relevant
R55     = 0;                        % Not relevant
Iz      = 1/12*m*(B^2+L^2)+m*CG(1)^2; % Moment of inertia for a plate 
R66     = sqrt(Iz*0.6/m);

% Rigid body inertia and coriolis matrix
[MRB_CO, CRB_CO] = rbody(m,R44,R55,R66,[0;0;r],CG);    % Finding MRB and CRB in CO       
MRB_CO  =   [MRB_CO(1,1),MRB_CO(1,2),MRB_CO(1,6);
             MRB_CO(2,1), MRB_CO(2,2), MRB_CO(2,6);
             MRB_CO(6,1), MRB_CO(6,2),MRB_CO(6,6)];          % 3-DOF 
        
CRB_CO  =   [CRB_CO(1,1),CRB_CO(1,2),CRB_CO(1,6);
             CRB_CO(2,1), CRB_CO(2,2), CRB_CO(2,6);
             CRB_CO(6,1), CRB_CO(6,2),CRB_CO(6,6)];             % 3-DOF 

% Added mass and coriolis matrix
M_A     =   [8e4, 0, 0;
             0 , 3.9e5, 1e6; %
             0, -5e5, 3e7];            % Rough estimates, based on navel vessel found in the MSS toolbox Xudot is increased
           % Rotation matrix from Body to NED    
C_A     =   m2c(M_A,nu);
C       =   CRB_CO+C_A;
M       =   MRB_CO+M_A;

%linear damping 

D       = zeros(3,3);
D(1,1)=M(1,1)/50; D(2,2)=M(2,2)/80; D(3,3)=M(3,3)/10; % Timeconstants from mail with Fossen

% Nonlinear surge damping
S       = 2*sum(A)+B*L+2*B*T*0.75;      % Wetted surfacearea, redduced with 0.75, as the Italien tug had disp of 659 and Sw og 392 0.64, but this has reduced sideareas.  ; 
kinVisc = 1e-6;
ur      =u; 
Rn      = L/kinVisc*abs(ur);
eps     =0.001;  
CR      = 0.02;              % Surface friction
Cf      = 0.075/((log10(Rn)-2)^2+eps)+CR;
Xdamp   = -0.5*rho*S*(1+0.25)*Cf*abs(ur)*ur;


% Cross-flow drag principle, with K defining the extra "Keel" 
Cd=zeros(1,length(xq));
for k=1:length(Cd)
    Check = B/(2*vq(k)); 
    if Check>4
        Cd(k)=0.5;
    else
        Cd(k)= Hoerner(B,vq(k));
    end
end

CdK = zeros(1,length(xq));
B2=1;
for k2=1:length(CdK)

    if  vqK(k2)<0.1
        CdK(k2)=0;
    else
        CdK(k2)= Hoerner(B2,vqK(k2));
    end
end

gain =ones(1,length(xq)); gain(2:end-1)=2; 
dx=0.5;
vr=v;
Ydamp=sum(-rho*0.5*dx*0.5*gain.*Cd.*vq.*abs(vr+xq*r).*(vr+xq*r));
Ndamp=sum(-rho*0.5*dx*0.5*gain.*Cd.*vq.*xq.*abs(vr+xq*r).*(vr+xq*r));

YKdamp=sum(-rho*0.5*dx*0.5*gain.*CdK.*vqK.*abs(vr+xq*r).*(vr+xq*r));
NKdamp=sum(-rho*0.5*dx*0.5*gain.*CdK.*vqK.*xq.*abs(vr+xq*r).*(vr+xq*r));

Ydamp = Ydamp+YKdamp;
Ndamp  =Ndamp+NKdamp;

Dn = -[Xdamp;Ydamp;Ndamp];
% ------- Creating dynamic functions------%
R       =   Rzyx(0,0,eta(3));    
eta_dot = R*nu;
% nu_dot  =M\(tau-C*nu-D*nu-Dn);

states = [eta;nu];

matlabFunction(M,C,D,Dn,eta_dot,'file','TugDynamicsCALC','vars',{states},'Comments',"in1=states=[xn;yn;psi;u;v;r]") 
%% Testing sliding friction
matlabFunction(M,C,D,Dn,'file','ContactFriction','vars',{nu},'Comments',"in1=states=[uM;vM;0]") 
% matlabFunction(eta_dot,nu_dot,'file','TugDynamicsCALC','vars',{states,input},'Comments',"in1=states=[xn;yn;psi;u;v;r], in2=inputs=[X;Y;N] A") 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%==========Tug 3-DOF PID controller====================% This is failing on
%hitting the side of the ship, since the integral action is useless as it
%can not get lower
% This is from Fossen (2021) p.532
 syms wb1 wb2 wb6 zeta1 zeta2 zeta6 
 Skew = Smtrx([0;0;r]);
 Wb=[wb1; wb2; wb6];
 Zeta = [zeta1; zeta2; zeta6];
 Omega_b = diag([wb1,wb2, wb6]);
 Z = diag([zeta1,zeta2, zeta6]);
 wn1=1/((1-2*zeta1^2+(4*zeta1^4-4*zeta1^2+2)^(1/2))^(1/2))*wb1;
 wn2=1/((1-2*zeta2^2+(4*zeta2^4-4*zeta2^2+2)^(1/2))^(1/2))*wb2;
 wn6=1/((1-2*zeta6^2+(4*zeta6^4-4*zeta6^2+2)^(1/2))^(1/2))*wb6;
Omega_n = diag([wn1,wn2,wn6]);
Kp = R*M*R.'*(Omega_n.'*Omega_n);
Kd = 2*R*M*R.'*Z*Omega_n-R*(C+D)*R.'+R*M*R.'*Skew;
Ki = 1/10*Kp*Omega_n;
matlabFunction(Kp,Kd,Ki,'file','TugPID_Gains','vars',{Wb,Zeta,psi,nu},'Comments','in1=bandwidths=[wb1;wb2;wb6], in2=damping ratios=[zeta1;zeta2;zeta6], in3=yaw angle=psi, in4=states=[u,v,r]') % Because of C, u,v,r has to be computed. 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%==========Tug heading PID controller====================%
% This is from Fossen (2021) p.529 and p.537
% Based on a first order nomoto model
% syms wbN zetaN
% wnN = 1/((1-2*zetaN^2+(4*zetaN^4-4*zetaN^2+2)^(1/2))^(1/2))*wbN;
% MN = M(3,3); DN = (D(3,3));
% KpN = wnN^2*MN; 
% KdN = 2*zetaN*wnN*MN-DN;
% KiN = wnN*KpN/10;
% matlabFunction(KpN,KdN,KiN,'file','TUG_HeadingPIDGains','vars',{wbN,zetaN}); 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%==========Tug sway PID controller====================%
% Based on a first order Nomoto model
syms wbY zetaY
wnY = 1/((1-2*zetaY^2+(4*zetaY^4-4*zetaY^2+2)^(1/2))^(1/2))*wbY;
MSway = M(2,2); DSway = D(2,2);
KpY = wnY^2*MSway; 
KdY = (2*zetaY*wnY*MSway-DSway);
KiY = wnY/10*KpY; 
matlabFunction(KpY,KdY,KiY,'file','TUG_SwayPIDGains','vars',{wbY,zetaY}); 


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%==========Tug surge PI controller====================%
% Based on a first order Nomoto model
syms wbX zetaX
wn = 1/((1-2*zetaX^2+(4*zetaX^4-4*zetaX^2+2)^(1/2))^(1/2))*wbX;
Msurge  = M(1,1); %8.7086e+05;     % value from symbolic tug calc matlab function
DSurge = D(1,1); %1.7417e+04;     % value from symbolic tug calc matlab function
Kp = 2*zetaX*wn*Msurge-DSurge;
Ki = Msurge*wn^2;
matlabFunction(Msurge,Kp,Ki,'file','TUG_SurgePI','vars',{wbX,zetaX}); 