clear;
 clc;
%  clear figure;

%%%%%%%%%%%%%%%%%%%%%set the figure parameters, not important to the kinematic simulation
% width = 3.15*2;
% height = 2*2;
% alw = 0.75;
% fsz = 11;
% lw = 1;
% msz = 6;
% 
% % The properties we've been using in the figures
% set(0,'defaultLineLineWidth',lw);   % set the default line width to lw
% set(0,'defaultLineMarkerSize',msz); % set the default line marker size to msz
% set(gca,'FontSize',fsz);
% 
% % Set the default Size for display
% defpos = get(0,'defaultFigurePosition');
% set(0,'defaultFigurePosition', [defpos(1) defpos(2)-1000 width*100, height*100]);
% 
% % Set the defaults for saving/printing to a file
% set(0,'defaultFigureInvertHardcopy','on'); % This is the default anyway
% set(0,'defaultFigurePaperUnits','inches'); % This is the default anyway
% defsize = get(gcf, 'PaperSize');
% left = (defsize(1)- width)/2;
% bottom = (defsize(2)- height)/2;
% defsize = [left, bottom, width, height];
% set(0, 'defaultFigurePaperPosition', defsize);
% set(gca,'ColorOrder',[0 0 1;1 0 0;0 0 0;0 0 1;1 0 0;0 0 0])

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Specifications of the SGR robot%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n = 30; % number of sections of the Soft Growing Robot
lg = 8; % horizontal gap between sheaths
ls = 12; % length of sheath segment
h = 4; % vertical gap between sheaths
lh = sqrt(lg^2+h^2); % calculate the hypotenuse of the iron line between sheaths
myVideo = VideoWriter('SGR_Bending_workspace_6stages'); %open video file
myVideo.FrameRate = 5;  %can adjust this, 5 - 10 works well for me

  
    

%%
%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Coordinate transformation for stage 1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for a=180:-1:2
        motion1 = 1:a; % input displacment of the tendon
        steps1 = size(motion1,2); 
        num_bends1 = floor(motion1./lh);%the number of sections that will be fully bended (maximum bending angle) give the intput motion.

        r1 = 38.5; % consider each section bend with this radius at stage 1

        theta = zeros(n,steps1); % build a matrix for angle of each sections for all differnt input motions at stage 1
        for i = 1:steps1 % calculate the bending angles of the bended joints given the input motion
            theta1(n-num_bends1(i),i) = (motion1(i) - num_bends1(i)*lh)/r1; % the angle of the joint which is not fully bended
            if num_bends1(i) >= 1 % at least one joint is fully bended.
                theta1(n:(-1):(n-num_bends1(i)+1),i) = lg/r1; %assgin the full bending angle lg/r to all those joints
            end
        end


        %%%%%%%%%%%section geometry%%%%%%%%%%%%%
        q1 = zeros(n,2*steps1); % set a matrix to contain oringinal coordinates of nodes with actuation 
        % q_orig=[[(ls+0.5*lg):(ls+lg):((ls+0.5*lg)+(n-1)*(ls+lg)) (2*ls+lg)+(n-1)*(ls+lg)]' zeros(n+1,1)];%origincal coordinates of nodes; (2*ls+lg)+(n-1)*(ls+lg) is the tip.
        q1_orig = [[0:(ls+lg):((n-1)*(ls+lg)) (ls+0.5*lg)+(n-1)*(ls+lg)]' zeros(n+1,1)]; % origincal coordinates of nodes; (2*ls+lg)+(n-1)*(ls+lg) is the tip.
        q1(1,1:2:end) = q1_orig(1,1); % coordinates of node 1 are fixed.
        q1(1,2:2:end) = q1_orig(1,2);


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Coordinate transformation%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for j=1:steps1 % determine motions given
            T_cumulative1 = eye(3);
            for i=1:n % determine section number
                if i==1 % i==1 represent element 1
                    T1 = [cos(theta1(i,j))  -sin(theta1(i,j))   0;
                          sin(theta1(i,j))   cos(theta1(i,j))   0;
                          0                  0                  1]; % transfromation matrix for 2d rotational matrix
                      T_cumulative1 = T_cumulative1*T1;
                      homogenous_corr1 = T_cumulative1*[(q1_orig(i+1,:)-q1_orig(i,:))';1];
                      q1(i+1,2*j-1) = homogenous_corr1(1);
                      q1(i+1,2*j) = homogenous_corr1(2);
                else
                    T1 = [cos(theta1(i,j))  -sin(theta1(i,j))   q1_orig(i,1)-q1_orig(i-1,1); % q_orig(i,1)-q_orig(i-1,1)represent x coordinate of node i-1 in frame i-1.
                          sin(theta1(i,j))   cos(theta1(i,j))   q1_orig(i,2)-q1_orig(i-1,2);
                          0                  0                  1];
                      T_cumulative1 = T_cumulative1*T1;
                      homogenous_corr1 = T_cumulative1*[(q1_orig(i+1,:)-q1_orig(i,:))';1]; % q_orig(i+1,:)-q_orig(i,:) represents coordinates of node i in frame i
                      q1(i+1,2*j-1) = homogenous_corr1(1);% q(i+1,:) represents coordinates of node i in the gloable frame;
                      q1(i+1,2*j) = homogenous_corr1(2);
                end
            end
        end


    %% 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Coordinate transformation for stage 2%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    motion2 = 1:a-1; % input displacment of the tendon
    steps2 = size(motion2,2); 
    num_bends2 = floor(motion2./lh);

    r2 = -38.5; % set stage 2 bending angle in radius

    for i=1:steps2 %calculate the bending angles of the bended joints
       theta2(n-num_bends2(i),i)=(motion2(i)-num_bends2(i)*lh)/r2;%the angle of the joint which is not fully bended
       if num_bends2(i)>=1 %at least one joint is fully bended.
           theta2(n:(-1):(n-num_bends2(i)+1),i)=lg/r2; %assgin the full bending angle lg/r to all those joints
       end
    end
    interval=1;


    %%%%%%%%%%%section geometry%%%%%%%%%%%%
    finalcol_tip_stage1 = steps1*2-1;
    finalcol2_tip_stage1 = steps1*2;
    q2_orig = q1(:,finalcol_tip_stage1:finalcol2_tip_stage1); % coordinates of tip node at stage 2

    q2 = zeros(n,2*steps2); % set a matrix
    q2(1,1:2:end)=q2_orig(1,1); % set x coordinates for the initial position at stage 2
    q2(1,2:2:end)=q2_orig(1,2); % set y coordinates for the initial position at stage 2


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Coordinate transformation%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for j=1:steps2 % determine motions given
        T_cumulative2=eye(3);    
        for i=1:n % determine section number
            if i==1 % i==1 represent element 1
                T2=[cos(theta2(i,j))  -sin(theta2(i,j))  0;
                    sin(theta2(i,j))   cos(theta2(i,j))  0;
                    0                 0                1]; % transfromation matrix for 2d rotational matrix
                T_cumulative2 = T_cumulative2*T2; 
                homogenous_corr2 = T_cumulative2*[(q2_orig(i+1,:)-q2_orig(i,:))';1];
                q2(i+1,2*j-1) = homogenous_corr2(1);
                q2(i+1,2*j) = homogenous_corr2(2);
            else             
               T2=[cos(theta2(i,j)) -sin(theta2(i,j)) q2_orig(i,1)-q2_orig(i-1,1);%q_orig(i,1)-q_orig(i-1,1)represent x coordinate of node i-1 in frame i-1.
                   sin(theta2(i,j)) cos(theta2(i,j))  q2_orig(i,2)-q2_orig(i-1,2);
                     0 0 1];
                T_cumulative2 = T_cumulative2*T2;
                homogenous_corr2 = T_cumulative2*[(q2_orig(i+1,:)-q2_orig(i,:))';1];%q_orig(i+1,:)-q_orig(i,:) represents coordinates of node i in frame i
                q2(i+1,2*j-1)=homogenous_corr2(1);% q(i+1,:) represents coordinates of node i in the gloable frame;
                q2(i+1,2*j)=homogenous_corr2(2);
            end
        end
    end
%     figure (1)
%     plot(q1(:,1:(2*interval):end),q1(:,2:(2*interval):end),'.-');%interval is used to adjust the motion finess to plot the robot's shape
%     hold on;
%     plot(q2(:,1:(2*interval):end),q2(:,2:(2*interval):end),'.-');
%     hold on;
%     xlabel('X coordinate(mm)');
%     ylabel('Y cooridnate(mm)');
%     axis([0 ((ls+0.5*lg)+(n-1)*(ls+lg))+0.5*(ls+lg) 0 50]);
%     axis([0 600 0 400])
%     daspect([1 1 1])
%     grid minor
%     print('SGR_6stages_trajectories','-dtiff','-r300')
    
    
    figure (2)
    plot(q1(31,1:(2*interval):end),q1(31,2:(2*interval):end),'.-');%interval is used to adjust the motion finess to plot the robot's shape
    hold on;
    plot(q2(31,1:(2*interval):end),q2(31,2:(2*interval):end),'.-');
    hold on;
    xlabel('X coordinate(mm)');
    ylabel('Y cooridnate(mm)');
    % axis([0 ((ls+0.5*lg)+(n-1)*(ls+lg))+0.5*(ls+lg) 0 50]);
    axis([0 600 0 400])
    daspect([1 1 1])
    grid minor
    print('SGR_6stages_workspace','-dtiff','-r300')

%     figure (3)
%     open(myVideo)
%     for i=1:size(q1,2)/2
%         x1=q1(:,2*i-1);
%         y1=q1(:,2*i); 
%         w1=plot(q1(31,1:(2*interval):end),q1(31,2:(2*interval):end),'.-');
%         hold on 
%         sgr1=plot(x1,y1,'.-r');
%         axis([0 600 0 400]) % axis limitation
%         daspect([1 1 1])
%         xlabel('X coordinate(mm)');
%         ylabel('Y cooridnate(mm)');
%         %pause(.01);
%         frame = getframe(gcf); %get frame
%         writeVideo(myVideo, frame);
%         delete(sgr1)
%     end
% 
%     for i=1:size(q2,2)/2
%         x2=q2(:,2*i-1);
%         y2=q2(:,2*i); 
%         w2=plot(q2(31,1:(2*interval):end),q2(31,2:(2*interval):end),'.-');
%         sgr2=plot(x2,y2,'.-r');
%         axis([0 600 0 400]) % axis limitation
%         daspect([1 1 1])
%         xlabel('X coordinate(mm)');
%         ylabel('Y cooridnate(mm)');
%         %pause(.01);
%         frame = getframe(gcf); %get frame
%         writeVideo(myVideo, frame);
%         delete(sgr2)
%     end
   
   end
% close(myVideo)
