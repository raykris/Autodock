# Autodock simulator - Automated tugboat-assisted docking of large vessels

The Autodock simulator is a Matlab/Simulink model for simulating automated docking operations with the assistance of tugboats. The simulator includes dynamics for tugboats and iron ore vessels, as well as time delays and interaction forces in pushing/pulling operations. 

The simulator is developed through the work:

	H. L. Ørke (2022). Autodock, Automated tugboat-assisted docking of large vessels. Master thesis. Department of Engineering 	Cybernetics, NTNU, Trondheim. URL: https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/3023120  

which serves as documentation for the simulator. Please include the following reference when using the simulator:

	H. L. Ørke, R. Kristiansen and J. T. Gravdahl. Autodock - a simulator for automated tugboat-assisted docking of large vessels. 	RL: https://github.com/raykris/Autodock. 

Copy contents of the directory Autodock/ to your computer and use "Add to Path -> Selected Folders and Subfolders" in Matlab.

 Necessary toolboxes and matlab Add-ons used (Matlab version r2023)
-

Matlab Add-ons:
- Simulink
- Stateflow
- Symbolic Math Toolbox (Used to create functions, may not be necessary, unless vessel dynamics are changed)
- Optimization Toolbox 

The MSS toolbox by Thor I. Fossen, download from Github https://github.com/cybergalactic/MSS. Follow download instructions, and make sure that the MSS folder is added to the Matlab path.


 RUN simulations
-


1. Open Main
2. Set initial conditions for bulk carrier in NED [x;y;psi;u;v;r] 
3. Choose number of tugboats:
3a. Copy the number of instances of Tug # as necessary, with out.Tug# for each.
3b. Connect all Tug "Force Tank" outputs through the sum, with inputs from BULK and Fe.
3c. Update thrust coefficient matrix and initial conditions for the time delay in the control system
4. Set towline lengths for the tugboats in instances of Tug #, connection at [17.5; 0] in Tug body
5. Set connection and towline point on bulk along the sides [<-100,100>;+23 or -23] (Bulk Body) 
6. Set initial conditions for Tug in NED [x;y;psi;u;v;r], based on limitations from point 2-5.

Initial position examples
-----------------------------------------------------------
Bulk dimension: 246x46
Tugs dimension: 35x11.5, where the bow has circular shape with radius 5.75m

ex.1: 
BulkStart 	-> zeros(6,1)
TugTowline 	-> 60 m
ConnectionBulk	-> [0;-23], 
TugStart	-> [0;-40.5;deg2rad(90);0;0;deg2rad(0)], Starts the bow tip midship in contact with the hull.
TugStart	-> [0;-100.5;deg2rad(90);0;0;deg2rad(0)], Starts the tugboat midship at the towline distance facing the hull.

ex.2: 
BulkStart 	-> [0;0;deg2rad(90);0;0;deg2rad(0)]
TugTowline 	-> 60 m
ConnectionBulk	-> [0;-23], 
TugStart	-> [40.5;0;deg2rad(180);0;0;0], Starts the bow tip midship in contact with the hull.
TugStart	-> [100.5;0;deg2rad(180);0;0;0], Starts the tugboat midship at the towline distance facing the hull.

TugStart position can be calculated from: BulkStart+RotationMatrix*[ConnectionBulk + (Tug origin relative to ConnectionBulk in bulk body)]:

[x_T;y_T]=[x_B;y_B]+[cos(psi_B), -sin(psi_B); sin(psi_B), cos(psi_B)]*[ConnectionBulk+TugOrigin_bulkbody]. 

TugStart angle is computed by BulkAngle - Tugboat rotation relative bulk (CCW positive around bulk z-axis (down))

Methods for calculating Tugstart position and angle are implemented in the file TugStart.m.
------------------------------------------------------------------------------------------------------

7. Decide the guidance system, set angles or use waypoints, (switch)
7a. Waypoints:
- See that the waypoints are loaded in workspace
- Start Bulk from the initial heading for best performance
8. Start simulation

Results:
- There are no scopes, but use logsignals and datainspector in Simulink.
- Run animation (see below) to see how the operation performs. 

Problem:
- If simulation runs slow after compiling, stop simulation and run animation to see tugboat placement


 Info on Matlab code
-

SymbolicTanker: Uses bulk carrier dynamcis to create codes
-----------------------------------------------------------
- TankerDynamics 	(Calculate the bulk carrier kinematics, [eta_dot,nu_dot])
- CourseAutoP 		(Calculate gains for the course autopilot, [Kp,Kd,Ki])		


SymbolicTugCalc: Uses tug dynamcis to create codes
-----------------------------------------------------------
- TugDynamicsCALC	(Calculate the Tugboat kinematics, [M,C,D,Dn,eta_dot])
- ContactFriction	(Find the required stick friction force, [M,C,D,Dn])	
- TugPID_Gains 		(Calculate gains for 3-DOF trajectory tracking, [Kp,Kd,Ki])
- TUG_SwayPIDGains      (Calculate gains for towangle control, [KpY,KdY,KiY])
- TUG_SurgePI		(Calculate gains for surge control, [Msurge,Kp,Ki])


Control allocation:
-----------------------------------------------------------
- QPsolver_Bulk: 	(Control allocation for bulk carrier (tugboat operation))
- TugAlloc_lbub: 	(Control allocation tugboat, azimuth thrusters with rotation)


Animations:
-----------------------------------------------------------
- AnimatingTugs 	(Animates the bulk carrier with x number of tugboats in 2D.)
- DrawTugShip  		(Create figures to the animation)

Waypoints: 
-----------------------------------------------------------
- Change_coordinates 	(Creates WP.mat that includes waypoints in NED for the course autopilot)
- pathplotter 		(Plot the bulk path in Narvik, coordinates in "Change_coordinates" must match)


