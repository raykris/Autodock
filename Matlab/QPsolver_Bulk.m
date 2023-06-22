function fe = QPsolver_Bulk(tau,Te,f0,dMax,f_max,f_min)
r       = size(Te,2);
n       = size(Te,1);
Df = ones(r,1)*dMax;
beta    = 1; 
P = [tau',f_min',f_max',beta,f0',Df']';
W = eye(r)*1; 
Q = eye(n)*1000;
PHI =   [W, zeros(r,n), zeros(r,1);
         zeros(n,r), Q, zeros(n,1);
         zeros(1,r), zeros(1,n), 0]; 
R = [zeros(r+n+1,n+2*r), [zeros(r+n,1);1],zeros(r+n+1,r),zeros(r+n+1,r)];
%----Equalities-----------%
A1 = [Te,-eye(n), zeros(n,1)]; 
C1 = [eye(n),zeros(n,2*r+1),zeros(n,r),zeros(n,r)];
%-----Inequalities---------%
A2 =    [-eye(r), zeros(r,n), zeros(r,1);
         eye(r), zeros(r,n), zeros(r,1);
         -eye(r), zeros(r,n),-ones(r,1);
         eye(r), zeros(r,n),-ones(r,1);
         eye(r), zeros(r,n),zeros(r,1);
         -eye(r), zeros(r,n), zeros(r,1)];
C2 = zeros(6*r,4*r+n+1); 
C2(1:r,n+1:n+r)=-eye(r);                % fmin
C2(r+1:2*r,r+n+1:n+2*r)=eye(r);         % Fmax  
C2(4*r+1:5*r,n+2*r+2:n+3*r+1)=eye(r); % F0   
C2(4*r+1:5*r,n+3*r+2:n+4*r+1)=eye(r); % Df  
C2(5*r+1:6*r,n+2*r+2:n+3*r+1)=-eye(r); % -F0   
C2(5*r+1:6*r,n+3*r+2:n+4*r+1)=eye(r); % Df  
options = optimoptions(	'quadprog',	'Algorithm','active-set');
X = quadprog(PHI,(R*P)',A2,C2*P,A1,C1*P,[],[],zeros(n+r+1,1), options);
fe=X(1:r);
end