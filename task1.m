clear all;
clc;
clear figure;

%%
%%%%%%%%%%%%%%%%%%%%%set the figure parameters, not important to the kinematic simulation
width = 3.15*2; height = 2*2; alw = 0.75; fsz = 11; lw = 1; msz = 6;
% The properties we've been using in the figures
set(0,'defaultLineLineWidth',lw);   % set the default line width to lw
set(0,'defaultLineMarkerSize',msz); % set the default line marker size to msz
set(gca,'FontSize',fsz);

% Set the default Size for display
defpos = get(0,'defaultFigurePosition');
set(0,'defaultFigurePosition', [defpos(1) defpos(2)-1000 width*100, height*100]);

% Set the defaults for saving/printing to a file
set(0,'defaultFigureInvertHardcopy','on'); % This is the default anyway
set(0,'defaultFigurePaperUnits','inches'); % This is the default anyway
defsize = get(gcf, 'PaperSize');
left = (defsize(1)- width)/2;
bottom = (defsize(2)- height)/2;
defsize = [left, bottom, width, height];
set(0, 'defaultFigurePaperPosition', defsize);
set(gca,'ColorOrder',[0 0 1;1 0 0;0 0 0;0 0 1;1 0 0;0 0 0])


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Specifications of the SGR robot%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n=30;%number of sections
r=30;%consider each section bend with this radius.
lg=8;% horizontal gap between sheaths
ls=12;% length of sheath segment
h=2;%vertical gap between sheaths

%% Forward motion

motion1=1:1:180;%input displacment of the cable or tendon
steps1=size(motion1,2);
theta1=zeros(n,steps1);%angle of each sections for all differnt input motions
num_bends1 = floor(motion1./sqrt(lg^2+h^2));%the number of sections that will be fully bended (maximum bending angle) give the intput motion.

for i=1:steps1 %calculate the bending angles of the bended joints given the input motion
   theta1(n-num_bends1(i),i)=(motion1(i)-num_bends1(i)*sqrt(lg^2+h^2))/r;%the angle of the joint which is not fully bended
   if num_bends1(i)>=1 %at least one joint is fully bended.
       theta1(n:(-1):(n-num_bends1(i)+1),i)=lg/r; %assgin the full bending angle lg/r to all those joints
   end
end
interval=1;
%%%%%%%%%%%section geometry%%%%%%%%%%%%%

q1 = zeros(n,2*steps1);%coordinates of nodes with actuation
% q_orig=[[(ls+0.5*lg):(ls+lg):((ls+0.5*lg)+(n-1)*(ls+lg)) (2*ls+lg)+(n-1)*(ls+lg)]' zeros(n+1,1)];%origincal coordinates of nodes; (2*ls+lg)+(n-1)*(ls+lg) is the tip.
q_orig1=[[0:(ls+lg):((n-1)*(ls+lg)) (ls+0.5*lg)+(n-1)*(ls+lg)]' zeros(n+1,1)];%origincal coordinates of nodes; (2*ls+lg)+(n-1)*(ls+lg) is the tip.
q1(1,1:2:end) = q_orig1(1,1);%coordinates of node 1 are fixed.
q1(1,2:2:end) = q_orig1(1,2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Coordinate transformation%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for j=1:steps1
    T_cumulative1=eye(3);    
    for i=1:n 
        if i==1 %i==1 represent element 1
            T1=[cos(theta1(i,j)) -sin(theta1(i,j)) 0;
               sin(theta1(i,j)) cos(theta1(i,j))  0;
                 0 0 1];
            T_cumulative1=T_cumulative1*T1;
            homogenous_corr1=T_cumulative1*[(q_orig1(i+1,:)-q_orig1(i,:))';1];
            q1(i+1,2*j-1)=homogenous_corr1(1);
            q1(i+1,2*j)=homogenous_corr1(2); 
        else             
           T1=[cos(theta1(i,j)) -sin(theta1(i,j)) q_orig1(i,1)-q_orig1(i-1,1);%q_orig(i,1)-q_orig(i-1,1)represent x coordinate of node i-1 in frame i-1.
               sin(theta1(i,j)) cos(theta1(i,j))  q_orig1(i,2)-q_orig1(i-1,2);
                 0 0 1];
            T_cumulative1=T_cumulative1*T1;
            homogenous_corr1=T_cumulative1*[(q_orig1(i+1,:)-q_orig1(i,:))';1];%q_orig(i+1,:)-q_orig(i,:) represents coordinates of node i in frame i
            q1(i+1,2*j-1)=homogenous_corr1(1);% q(i+1,:) represents coordinates of node i in the gloable frame;
            q1(i+1,2*j)=homogenous_corr1(2);
        end
    end
end

%% Reverse motion
motion=180:-1:1; %input displacmnt of the cable or tendon
steps=size(motion,2);
theta=zeros(n,steps);%angle of each sections for all differnt input motions
num_bends = floor(motion./sqrt(lg^2+h^2));%the number of sections that will be fully bended (maximum bending angle) give the intput motion.

for i=steps:-1:1 %calculate the bending angles of the bended joints given the input motion
   theta(n-num_bends(i),i)=(motion(i)-num_bends(i)*sqrt(lg^2+h^2))/r;%the angle of the joint which is not fully bended
   if num_bends(i)>=1 %at least one joint is fully bended.
       theta(n:(-1):(n-num_bends(i)+1),i)=lg/r; %assgin the full bending angle lg/r to all those joints
   end
end
interval=1;
%%%%%%%%%%%section geometry%%%%%%%%%%%%%

q = zeros(n,2*steps);%coordinates of nodes with actuation
% q_orig=[[(ls+0.5*lg):(ls+lg):((ls+0.5*lg)+(n-1)*(ls+lg)) (2*ls+lg)+(n-1)*(ls+lg)]' zeros(n+1,1)];%origincal coordinates of nodes; (2*ls+lg)+(n-1)*(ls+lg) is the tip.
q_orig=[[0:(ls+lg):((n-1)*(ls+lg)) (ls+0.5*lg)+(n-1)*(ls+lg)]' zeros(n+1,1)];%origincal coordinates of nodes; (2*ls+lg)+(n-1)*(ls+lg) is the tip.
q(1,1:2:end) = q_orig(1,1);%coordinates of node 1 are fixed.
q(1,2:2:end) = q_orig(1,2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Coordinate transformation%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for j=1:steps 
    T_cumulative=eye(3);    
    for i=1:n 
        if i==1 %i==1 represent element 1
            T=[cos(theta(i,j)) -sin(theta(i,j)) 0;
               sin(theta(i,j)) cos(theta(i,j))  0;
                 0 0 1];
            T_cumulative=T_cumulative*T;
            homogenous_corr=T_cumulative*[(q_orig(i+1,:)-q_orig(i,:))';1];
            q(i+1,2*j-1)=homogenous_corr(1);
            q(i+1,2*j)=homogenous_corr(2); 
        else             
           T=[cos(theta(i,j)) -sin(theta(i,j)) q_orig(i,1)-q_orig(i-1,1);%q_orig(i,1)-q_orig(i-1,1)represent x coordinate of node i-1 in frame i-1.
               sin(theta(i,j)) cos(theta(i,j))  q_orig(i,2)-q_orig(i-1,2);
                 0 0 1];
            T_cumulative=T_cumulative*T;
            homogenous_corr=T_cumulative*[(q_orig(i+1,:)-q_orig(i,:))';1];%q_orig(i+1,:)-q_orig(i,:) represents coordinates of node i in frame i
            q(i+1,2*j-1)=homogenous_corr(1);% q(i+1,:) represents coordinates of node i in the gloable frame;
            q(i+1,2*j)=homogenous_corr(2);
        end
    end
end

%% forward motion graph
figure (1)
plot(q1(:,1:(2*interval):end),q1(:,2:(2*interval):end),'.-');%interval is used to adjust the motion finess to plot the robot's shape
hold on
plot(q1(31,1:(2*interval):end),q1(31,2:(2*interval):end),'.-r');
xlabel('X coordinate(mm)');
ylabel('Y cooridnate(mm)');
% axis([0 ((ls+0.5*lg)+(n-1)*(ls+lg))+0.5*(ls+lg) 0 50]);
axis([0 600 0 150])
daspect([1 1 1])
grid minor
print('Test_forward_tip_workspace','-dtiff','-r300') 

%% reverse motion
figure (2)
plot(q(:,1:(2*interval):end),q(:,2:(2*interval):end),'.-');%interval is used to adjust the motion finess to plot the robot's shape
hold on
plot(q(31,1:(2*interval):end),q(31,2:(2*interval):end),'.-b');
xlabel('X coordinate(mm)');
ylabel('Y cooridnate(mm)');
% axis([0 ((ls+0.5*lg)+(n-1)*(ls+lg))+0.5*(ls+lg) 0 50]);
axis([0 600 0 150])
daspect([1 1 1])
grid minor
print('Test_reverse_tip_workspace','-dtiff','-r300') 

%%
figure (3)
myVideo = VideoWriter('myVideoFile_Forward'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)
for i=1:size(q1,2)/2
    x2=q1(:,2*i-1);
    y2=q1(:,2*i); 
    plot(x2,y2,'.-r');
%     axis equal
    axis([0 600 0 150])
    daspect([1 1 1])
    xlabel('X coordinate(mm)');
    ylabel('Y cooridnate(mm)');
    pause(.01);
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end
close(myVideo)

%% Reverse 
figure (4)
myVideo = VideoWriter('myVideoFile_reverse'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)
for i=1:size(q,2)/2
    x1=q(:,2*i-1);
    y1=q(:,2*i); 
    plot(x1,y1,'.-r');
%     axis equal
    axis([0 600 0 150])
    daspect([1 1 1])
    xlabel('X coordinate(mm)');
    ylabel('Y cooridnate(mm)');
    pause(.01);
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end
close(myVideo)

%% combine video
figure (5)
myVideo = VideoWriter('myVideoFile_combine'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)
for i=1:size(q1,2)/2
    x2=q1(:,2*i-1);
    y2=q1(:,2*i); 
    plot(x2,y2,'.-r');
%     axis equal
    axis([0 600 0 150])
    daspect([1 1 1])
    xlabel('X coordinate(mm)');
    ylabel('Y cooridnate(mm)');
    pause(.01);
    grid on
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end
for i=1:size(q,2)/2
    x1=q(:,2*i-1);
    y1=q(:,2*i); 
    plot(x1,y1,'.-r');
%     axis equal
    axis([0 600 0 150])
    daspect([1 1 1])
    xlabel('X coordinate(mm)');
    ylabel('Y cooridnate(mm)');
    pause(.01);
    grid on
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end

close(myVideo)