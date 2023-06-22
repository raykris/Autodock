
%% Plot path coordinates (Narvik)
figure(1)
Tank = out.Tank(1:3,:);
x=Tank(1,:);
y= Tank(2,:);
pathplotter(x,y)

%% Animation of tugboats and vessel motion
clear tug; 
figure(2)
Tank = out.Tank(1:3,1,:);
TugA = out.TugA(1:8,1,:);
TugB = out.TugB(1:8,1,:);
%TugC = out.TugC(1:8,1,:);
%TugD = out.TugD(1:8,1,:);
N = length(TugA);
view = 20; % lower to zoom, 
timeincrement = 40; % Matched with timestep: 40 = 1/0.025. Increase for faster animation. 
for i=1:timeincrement:N
    clf;
  tank = Tank(:,:,i);
tug{1}=TugA(:,:,i);
tug{2}=TugB(:,:,i);
%tug{3}=TugC(:,:,i);
%tug{4}=TugD(:,:,i);
DrawTugShip(tank,tug,view);
drawnow 
end



