function [eta_dot,nu_dot] = TankerDynamics(in1,in2)
%TankerDynamics
%    [ETA_DOT,NU_DOT] = TankerDynamics(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    06-Jun-2022 18:36:38

%in1=states=[xn;yn;psi;u;v;r], in2=inputs=[X;Y;N]
N = in2(3,:);
X = in2(1,:);
Y = in2(2,:);
psi = in1(3,:);
r = in1(6,:);
u = in1(4,:);
v = in1(5,:);
t2 = cos(psi);
t3 = sin(psi);
eta_dot = [t2.*u-t3.*v;t3.*u+t2.*v;r];
if nargout > 1
    et1 = N.*(-7.752047608784612e-12)+Y.*6.903810895587011e-9+r.*2.187992208712281e-1;
    et2 = v.*(-5.290310822791334e-2)-r.*u.*6.762560471445275e-1;
    et3 = u.*v.*3.635206040380925e-4;
    et4 = N.*1.69584499999743e-12-Y.*7.751648849705366e-12-r.*4.786471696811206e-2;
    et5 = v.*5.939999287971147e-5-r.*u.*3.635309334580809e-4;
    et6 = u.*v.*(-7.952409864658935e-5);
    nu_dot = [X.*1.013157694328251e-8-u.*6.25e-3+r.*v.*1.475105050452329+r.^2.*6.742827601176145;et1+et2+et3;et4+et5+et6];
end
