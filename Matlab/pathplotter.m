
function pathplotter(x,y)
% pathplotter(x,y) converts NE-positions to longitude and latitude coordinates
% and plots these coordinates over a map using geoplot.
%
% x = position in x-direction (North)         (m)
% y = position in y-direction (East)          (m)
%

% Longitude and Latitude coordinates of the waypoints
WP1_long = 17.3670 ; WP1_lat = 68.4300;
WP2_long = 17.3766 ; WP2_lat = 68.4282;
WP3_long = 17.3850 ; WP3_lat = 68.4249;
WP4_long = 17.3903 ; WP4_lat = 68.4244;
WP5_long = 17.3947 ; WP5_lat = 68.4256;
WP6_long = 17.3954 ; WP6_lat = 68.4272;
WP7_long = 17.3931 ; WP7_lat = 68.4290; 
WP8_long = 17.3912 ; WP8_lat = 68.4293; 

WP_long = [WP1_long, WP2_long, WP3_long,WP4_long, WP5_long, WP6_long, WP7_long, WP8_long ];
WP_lat = [WP1_lat, WP2_lat, WP3_lat,WP4_lat, WP5_lat, WP6_lat, WP7_lat, WP8_lat];
l_0 = deg2rad(WP1_long); mu_0 = deg2rad(WP1_lat); h_ref = 0;

geoplot(WP_lat,WP_long,'x--','color','#D95319','linewidth',1)
hold on 
text(WP_lat(1),WP_long(1),'\leftarrow Start')

[l,mu,~] = flat2llh(x,y,zeros(1,length(x)),l_0,mu_0,h_ref);
l = rad2deg(l); mu = rad2deg(mu);

geoplot(mu,l,'-','color','#0072BD', 'linewidth',1)
legend('Generated path','Bulk carrier','FontSize', 10, 'location', 'NorthEast');
end
 
