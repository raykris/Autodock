
%% Defining waypoints for vessel motion
clear all; close all; clc
WP1_long = 17.3670 ; WP1_lat = 68.4300;
WP2_long = 17.3766 ; WP2_lat = 68.4282;
WP3_long = 17.3850 ; WP3_lat = 68.4249;
WP4_long = 17.3903 ; WP4_lat = 68.4244;
WP5_long = 17.3947 ; WP5_lat = 68.4256;
WP6_long = 17.3954 ; WP6_lat = 68.4272;
WP7_long = 17.3931 ; WP7_lat = 68.4290; 
WP8_long = 17.3912 ; WP8_lat = 68.4293; 


WP_long = deg2rad([WP1_long, WP2_long, WP3_long,WP4_long, WP5_long, WP6_long, WP7_long, WP8_long ]);
WP_lat = deg2rad([WP1_lat, WP2_lat, WP3_lat,WP4_lat, WP5_lat, WP6_lat, WP7_lat, WP8_lat]);

l_0 = deg2rad(WP1_long); mu_0 = deg2rad(WP1_lat); h_ref = 0;
h=zeros(1,length(WP_long));
% [l,mu,~] = flat2llh(x,y,zeros(1,length(x)),l_0,mu_0,h_ref);

[x,y,z] = llh2flat(WP_long,WP_lat,h,l_0,mu_0,h_ref);
x=x-x(1);
y=y-y(1);
WP=[x;y];
save('WP.mat','WP');