clear all; close all; clc;
%bag_name = '2020-12-15-19-31-38.bag';
bag_name = '2020-12-15-19-17-16.bag';
%bag_name = '2020-12-15-19-31-38.bag';


topics2read = {'/joint_states',...
         '/mpc/debug/opt_traj',...
         '/mpc/debug/ref_traj',...
         '/mpc/debug/cmd',...
         '/rt/olc_cmd',...
         '/rt/odom'};
[bag_data, topics_read] = bagReader(bag_name,topics2read);
%[bag_data, topics_read] = bagReader(bag_name);
topics_read = topics_read';

% % Separate data
joint_states_bg = bag_data{1,1};
mpc_opt_traj_bg = bag_data{2,1};
mpc_ref_traj_bg = bag_data{3,1};
mpc_cmd_bg = bag_data{4,1};
rt_olc_cmd_bg = bag_data{5,1};
rt_odom_bg = bag_data{6,1};

rt_odom.xPos = rt_odom_bg{:,1};
rt_odom.yPos = rt_odom_bg{:,3};
rt_odom.xAng = rt_odom_bg{:,8};
rt_odom.yAng = rt_odom_bg{:,4};
rt_odom.time = rt_odom_bg{:,5} - rt_odom_bg{1,5};

mpc_cmd.xAng = mpc_cmd_bg{:,3};
mpc_cmd.yAng = mpc_cmd_bg{:,2};
mpc_cmd.xVel = mpc_cmd_bg{:,4};
mpc_cmd.yVel = mpc_cmd_bg{:,5};
mpc_cmd.time = mpc_cmd_bg{:,1} - mpc_cmd_bg{1,1};

mpc_ref.xPos = mpc_ref_traj_bg{:,1};
mpc_ref.yPos = mpc_ref_traj_bg{:,3};
mpc_ref.xAng = mpc_ref_traj_bg{:,8};
mpc_ref.yAng = mpc_ref_traj_bg{:,4};
mpc_ref.xVel = mpc_ref_traj_bg{:,6};
mpc_ref.yVel = mpc_ref_traj_bg{:,9};
mpc_ref.time = mpc_ref_traj_bg{:,5} - mpc_ref_traj_bg{1,5};

%% Sections of plot
% RT ODOM
figure
subplot(2,2,1)
plot(rt_odom.time, rt_odom.xPos)
ylabel('XPos [m]')

subplot(2,2,2)
plot(rt_odom.time,rt_odom.yPos)
ylabel('YPos [m]')

subplot(2,2,3)
plot(rt_odom.time,rt_odom.xAng)
ylabel('XAng [rad]')

subplot(2,2,4)
plot(rt_odom.time,rt_odom.yAng)
ylabel('YAng [rad]')
suptitle('RT Odom')

% MPC CMD
figure
subplot(2,2,1)
plot(mpc_cmd.time, mpc_cmd.xAng)
ylabel('XAng [rad]')

subplot(2,2,2)
plot(mpc_cmd.time, mpc_cmd.yAng)
ylabel('YAng [rad]')

subplot(2,2,3)
plot(mpc_cmd.time, mpc_cmd.xVel)
ylabel('XVel [m/s]')

subplot(2,2,4)
plot(mpc_cmd.time, mpc_cmd.yVel)
ylabel('YVel[m/s]')
suptitle('MPC CMD')


% MPC REF
figure
subplot(2,2,1)
plot(mpc_ref.time, mpc_ref.xAng)
ylabel('XAng [rad]')

subplot(2,2,2)
plot(mpc_ref.time, mpc_ref.yAng)
ylabel('YAng [rad]')

subplot(2,2,3)
plot(mpc_ref.time, mpc_ref.xPos)
ylabel('XPos [m]')

subplot(2,2,4)
plot(mpc_ref.time, mpc_ref.yPos)
ylabel('YPos [m]')
suptitle('MPC Ref')


% %--------------------------------------------------------------------
%Format plot
%set(h_legend,'FontSize',22,'FontWeight','normal');
%set(h_legend2,'FontSize',20,'FontWeight','Bold');
%legend boxoff 

set(gca,'FontSize',22,'FontWeight','Normal','Color','w');%'PlotBoxAspectRatioMode','manual','PlotBoxAspectRatio',[1,0.5,1])
set(x_label,'FontSize',24,'FontWeight','Bold');
%set(x_label2,'FontSize',24,'FontWeight','Bold');
set(y_label,'FontSize',24,'FontWeight','Bold');

figPos = [0, 0, 1200, 450];
set(gcf,'color','white','position',figPos,'PaperUnits','points','PaperPosition',figPos)

%print('-depsc2','-r600',plotName)
%print('-dpng','-r1200',strcat(plotName,'.png'));