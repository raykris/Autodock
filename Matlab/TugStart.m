
%% Method for calculating tugboat starting positions and angle

% Bulk initial values
bulkAngle_n = 101;%90;
Bulk_0 = [0;0;deg2rad(bulkAngle_n);0;0;deg2rad(0)];

bulkstart = [Bulk_0(1); Bulk_0(2)];
psi_B = Bulk_0(3);
R_mat = [cos(psi_B), -sin(psi_B); sin(psi_B), cos(psi_B)];

% Tug A
conbulkA = [80; -23];
tugoriginA_b = [0; -17.5]; % Connection point in tugboat body [17.5 0]
tugstartA_n = bulkstart + R_mat*(conbulkA + tugoriginA_b);
tugAngleA_b = -90;
tugAngleA = bulkAngle_n - tugAngleA_b;

outstring = strcat('Tugstart A: [', num2str(tugstartA_n(1)), {'; '}, num2str(tugstartA_n(2)), '; degtorad(', num2str(tugAngleA), ')]');
disp(outstring)

% Tug B
conbulkB = [-80; -23];
tugoriginB_b = [0; -17.5]; % Connection point in tugboat body [17.5 0]
tugstartB_n = bulkstart + R_mat*(conbulkB + tugoriginB_b);
tugAngleB_b = -90;
tugAngleB = bulkAngle_n - tugAngleB_b;

outstring = strcat('Tugstart B: [', num2str(tugstartB_n(1)), {'; '}, num2str(tugstartB_n(2)), '; degtorad(', num2str(tugAngleB), ')]');
disp(outstring)

% Tug C
% conbulkC = [80; 23];
% tugoriginC_b = [0; 17.5]; % Connection point in tugboat body [17.5 0]
% tugstartC_n = bulkstart + R_mat*(conbulkC + tugoriginC_b);
% tugAngleC_b = 90;
% tugAngleC = bulkAngle_n - tugAngleC_b;
% 
% outstring = strcat('Tugstart C: [', num2str(tugstartC_n(1)), {'; '}, num2str(tugstartC_n(2)), '; degtorad(', num2str(tugAngleC), ')]');
% disp(outstring)

% Tug D
% conbulkD = [-80; 23];
% tugoriginD_b = [0; 17.5]; % Connection point in tugboat body [17.5 0]
% tugstartD_n = bulkstart + R_mat*(conbulkD + tugoriginD_b);
% tugAngleD_b = 90;
% tugAngleD = bulkAngle_n - tugAngleD_b;
% 
% outstring = strcat('Tugstart D: [', num2str(tugstartD_n(1)), {'; '}, num2str(tugstartD_n(2)), '; degtorad(', num2str(tugAngleD), ')]');
% disp(outstring)
