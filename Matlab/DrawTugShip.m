%% Drawing tugboats and vessel for every time instant in the animation
function DrawTugShip(Ship, Tug, View)
FA=1;
    EC = 'k';
    scale = 1;


    %Cube
    vert{1} =   [ -23, -123, 0;  %1
                   -23, 100, 0;  %2
                   0, 123, 0;  %3
                   23, 100, 0;   %4
                  23, -123, 0];  %5
              
vert{1} =vert{1}*Rzyx(0,0,Ship(3)); 
    fac{1} = [1, 2, 3, 4,5];

NumTugs = length(Tug);
if NumTugs >0; 
for T = 1:NumTugs
        state=Tug{T};
        i=1+T;
        vert{i} =   [ -5.75, -17.5, 0;  %1
                    -5.75, 11.75, 0;    %2
                    -4.9796, 14.6250,0; %3 
                    -4.0659, 15.8159,0; %4
                    -2.8750, 16.7296,0; %5
                    0, 17.5,0;          %6
                    2.8750, 16.7296, 0; %7
                    4.0659, 15.8159, 0; %8
                    4.9796, 14.6250, 0; %9
                    5.75, 11.75, 0;     %10
                    5.75, -17.5, 0];    %11
               vert{i}= vert{i}*Rzyx(0,0,state(3));
              fac{i} = [1, 2, 3, 4, 5, 6, 7, 8,9,10,11];     
              pos{i} = ones(length(vert{i}),1)*[state(2),state(1),0];
% Thruster port side
          vertP{2*T-1}    =[0,0,0;           % 1
                        -2, -10,0;            % 2
                         2, -10,0];           % 3
         vertP{2*T-1}    = vertP{2*T-1}*Rzyx(0,0,state(3))*Rzyx(0,0,state(7));
         facP{2*T-1}     = [1,2,3];
         posP{2*T-1} = ones(length(vertP{2*T-1}),1)*[state(2),state(1),0]+[-3,-13.9,0]*Rzyx(0,0,state(3));
         
% Thruster starbord side
          vertP{2*T}    =[0,0,0;           % 1
                        -2, -10,0;            % 2
                         2, -10,0];           % 3
         vertP{2*T}    = vertP{2*T}*Rzyx(0,0,state(3))*Rzyx(0,0,state(8));
         facP{2*T}     = [1,2,3];
         posP{2*T} = ones(length(vertP{2*T}),1)*[state(2),state(1),0]+[3,-13.9,0]*Rzyx(0,0,state(3));
end
end



%     vert{2} =   [ -1, -1, 0;  %1
%                    1, -1, 0;  %2
%                    1,  1, 0;  %3
%                   -1,  1, 0]; %4
% 
%     fac{2} = [1 2 3 4];
     
%     vert{2} = vert{1}/4;fac{2} = fac{1};      
          
%     Lrail = 15;

    %Rail
%     a = 1;
%     vert{3} = [-Lrail,-a,-0.1;
%                -Lrail, a,-0.1;
%                 Lrail, a,-0.1;
%                 Lrail,-a,-0.1];
%     fac{3} = [1,2,3,4];
    pos{1} = ones(length(vert{1}),1)*[Ship(2),Ship(1),0];
%     pos{2} = ones(length(vert{2}),1)*[Tug(2),Tug(1),0]; 
%     pos{3} = 0;
    
    Col = {[0.01,0.4,0.4],[0.4,0.4,0.4],[0.4,0.4,0.4],[0.4,0.4,0.4],[0.4,0.4,0.4],[0.4,0.4,0.4]};
    ColP = {[1,0,0],[0,1,0],[1,0,0],[0,1,0],[1,0,0],[0,1,0],[1,0,0],[0,1,0],[1,0,0],[0,1,0]};

    for k = 1:1+NumTugs
        h{k} = patch('faces',fac{k},'vertices',scale*vert{k}+pos{k});
        set(h{k},'FaceColor',Col{k},'edgecolor',EC,'facealpha',FA,'FaceLighting','gouraud','SpecularStrength',1,'Diffusestrength',0.5,'AmbientStrength',0.7,'SpecularExponent',5)
    end

    for KK = 1:2*NumTugs
        hP{KK} = patch('faces',facP{KK},'vertices',scale*vertP{KK}+posP{KK});
        set(hP{KK},'FaceColor',ColP{KK},'edgecolor',EC,'facealpha',FA,'FaceLighting','gouraud','SpecularStrength',1,'Diffusestrength',0.5,'AmbientStrength',0.7,'SpecularExponent',5)
    end
 
    
    %% Add lights
   %light('Position',[1 3 Lrail]);
%light('Position',[-3 0 Lrail]);
%    light('Position',[ 0 0 Lrail]);
%    light('Position',[-Lrail 0 Lrail]);
    hold off
    axis equal
    set(gca,'visible','off');
    set(gca,'xtick',[],'ytick',[],'ztick',[]);
    
    %campos([100, 0, 1000])
    campos('auto')
    cc = campos;
    campos([cc(1), cc(2), 1000])
    camtarget([cc(1), cc(2), 0])
    camva(View)
    camproj('perspective')


end

