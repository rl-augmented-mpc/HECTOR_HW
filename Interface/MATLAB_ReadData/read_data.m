clear; close all; clc;
%addpath("include");

%% Flags and Data Selection %%

% data selection
realorsim = 'real';
date = '2024-03-26';
time = '12-24-43';

% Plot Selection
% if 1, display the plot, if 0, do not.

% Spatial data
com_rpy_flag        = 1; % display also for omega in the same plot
com_pos_flag        = 1; % also for vel, acc
foot_left_rpy_flag  = 0; % also for omega
foot_left_pos_flag  = 1; % also for vel 
foot_right_rpy_flag = 0; % also for omega
foot_right_pos_flag = 1; % also for vel 

%sensor data
gyro_flag           = 0;
acc_flag            = 0;
left_joint_flag     = 1;
right_joint_flag    = 1;

%controller data;
GRF_left_flag       = 1;
GRM_left_flag       = 1;
GRF_right_flag      = 1;
GRM_right_flag      = 1;
joint_cmd_flag      = 1; % cmd torque, pos, vel in one
ctrl_solver_flag    = 0; % controller solving time, cost, iteration
FSM_flag = 0;

%% Addressing files
cd(['../results/' realorsim date '_' time]);

% 2000: 0.00647, 0.00226, 0.00810
% 10000: 0.00408, 0.00242, 0.00656
% 10000_Dgain: 0.00453, 0.00202, 0.00717 *
% 10000_input: 0.00597, 0.00198, 0.00844
% 10000_input_Dgain: 0.00695, 0.00206, 0.00961

% 0.0287, 0.0329, 0.105
% 0.0283, 0.0288, 0.101



sensor_data = readmatrix("sensor_data.txt");
    idx_imu = 1;
    idx_imu_bias = idx_imu + 6;
    idx_joint_pos = idx_imu_bias + 6;
    idx_joint_vel = idx_joint_pos + 10;
    idx_joint_tau = idx_joint_vel + 10;

estimated_data = readmatrix("estimated_data.txt");
    idx_com_rpy   = 1;
    idx_com_omega = idx_com_rpy+3;
    idx_com_pos   = idx_com_omega+3;
    idx_com_vel   = idx_com_pos+3;
    idx_com_acc   = idx_com_vel+3;
    idx_foot_left_rpy = idx_com_acc+3;
    idx_foot_left_omega = idx_foot_left_rpy+3;
    idx_foot_left_pos = idx_foot_left_omega+3;
    idx_foot_left_vel = idx_foot_left_pos+3;
    idx_foot_right_rpy = idx_foot_left_vel+3;
    idx_foot_right_omega = idx_foot_right_rpy+3;
    idx_foot_right_pos = idx_foot_right_omega+3;
    idx_foot_right_vel = idx_foot_right_pos+3;
    idx_foot_left_contact = idx_foot_right_vel+3;
    idx_foot_right_contact = idx_foot_left_contact+1;

controller_data = readmatrix("controller_data.txt");
    idx_GRF_left = 1;
    idx_GRF_right = idx_GRF_left + 3;
    idx_GRM_left = idx_GRF_right + 3;
    idx_GRM_right = idx_GRM_left + 3; %9
    
    idx_joint_cmd_tau = idx_GRM_right + 3;
    idx_joint_cmd_pos = idx_joint_cmd_tau + 10;
    idx_joint_cmd_vel = idx_joint_cmd_pos + 10; %26
    
    idx_ctrl_solver_time = idx_joint_cmd_vel + 10;
    idx_ctrl_solver_cost = idx_ctrl_solver_time + 1;
    idx_ctrl_solver_iter = idx_ctrl_solver_cost + 1; %12
    
    idx_ref_com_rpy = idx_ctrl_solver_iter+1;
    idx_ref_com_omega = idx_ref_com_rpy +3;
    idx_ref_com_pos   = idx_ref_com_omega+3;
    idx_ref_com_vel   = idx_ref_com_pos+3;
    idx_ref_foot_left_rpy = idx_ref_com_vel+3;
    idx_ref_foot_left_omega = idx_ref_foot_left_rpy+3;
    idx_ref_foot_left_pos = idx_ref_foot_left_omega+3;
    idx_ref_foot_left_vel = idx_ref_foot_left_pos+3;
    idx_ref_foot_right_rpy = idx_ref_foot_left_vel+3;
    idx_ref_foot_right_omega = idx_ref_foot_right_rpy+3;
    idx_ref_foot_right_pos = idx_ref_foot_right_omega+3;
    idx_ref_foot_right_vel = idx_ref_foot_right_pos+3;
    idx_ref_foot_left_contact = idx_ref_foot_right_vel+3;
    idx_ref_foot_right_contact = idx_ref_foot_left_contact+1; %38

    idx_soln_com_rpy = idx_ref_foot_right_contact + 1;
    idx_soln_com_omega = idx_soln_com_rpy + 3;
    idx_soln_com_pos   = idx_soln_com_omega + 3;
    idx_soln_com_vel   = idx_soln_com_pos + 3;
    idx_soln_foot_left_pos = idx_soln_com_vel + 3;
    idx_soln_foot_left_vel = idx_soln_foot_left_pos + 3;
    idx_soln_foot_right_pos = idx_soln_foot_left_vel + 3;
    idx_soln_foot_right_vel = idx_soln_foot_right_pos + 3; %25

    idx_controller_data_end = idx_soln_foot_right_vel + 3; %3


% Assume that all the three data were recorded based on the same timestep
FSM_dt = 0.001;
starting_trash_data_count = 0;
finishing_trash_data_count = 0;
Timestep_length = size(sensor_data,1); % can be any among the threes.
Timesteps = 1 + starting_trash_data_count:1:Timestep_length - finishing_trash_data_count;
Time = FSM_dt .* Timesteps;



%% Contact Timespan Calculation

idx = 0;
for i = Timesteps
    idx = idx + 1;

    if controller_data(i, idx_ref_foot_left_contact)
        contact_plan(1,1,idx) = Time(i);
        bar_contact_plan(1,1,idx) = 1;
    end
    
    if controller_data(i, idx_ref_foot_right_contact)
        contact_plan(2,1,idx) = Time(i);
        bar_contact_plan(2,1,idx) = 1;
    end

    if estimated_data(i, idx_foot_left_contact)
        contact_est(1,1,idx) = Time(i);
        bar_contact_est(1,1,idx) = 1;
    end

    if estimated_data(i, idx_foot_right_contact)
        contact_est(2,1,idx) = Time(i);
        bar_contact_est(2,1,idx) = 1;
    end
end


%% Plotting


% For spatial data, reference traj, solution traj, true(estimated) traj should be displayed in a plot
% All plots should be colored with contact(swing/stable stance/sliding stance) at time : planned and actual
fig_num = 1;

%% com rpy & omega
if com_rpy_flag
    figure(fig_num);

    subplot(2,3,1); 
    plot(Time, estimated_data(Timesteps, idx_com_rpy), "k-" ); hold on;
    plot(Time, controller_data(Timesteps, idx_ref_com_rpy), "r--" );
    %plot(Time, controller_data(Timesteps, idx_soln_com_rpy), "b.-" ); hold off;
    grid minor; % other entries include 'lf plan', 'lf est', 'rf plan', 'rf est'
    xlabel('Time (s)'); ylabel('Roll (rad)'); title('Body Roll');
    txt1 = ['estimated, RMSE: ' num2str( rms(estimated_data(Timesteps, idx_com_rpy)-controller_data(Timesteps, idx_ref_com_rpy)))];
    legend(txt1,'ref','soln', 'location', 'northwest');
    
    subplot(2,3,2); hold on;
    plot(Time, estimated_data(Timesteps, idx_com_rpy + 1), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_com_rpy + 1), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_com_rpy + 1), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'southwest');
    xlabel('Time (s)'); ylabel('Pitch (rad)'); title('Body Pitch');
    txt1 = ['estimated, RMSE: ' num2str( rms(estimated_data(Timesteps, idx_com_rpy+1)-controller_data(Timesteps, idx_ref_com_rpy+1)))];
    legend(txt1,'ref','soln', 'location', 'northwest');

    subplot(2,3,3); hold on;
    plot(Time, estimated_data(Timesteps, idx_com_rpy + 2), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_com_rpy + 2), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_com_rpy + 2), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'southwest');
    xlabel('Time (s)'); ylabel('Yaw (rad)'); title('Body Yaw');
    txt1 = ['estimated, RMSE: ' num2str( rms(estimated_data(Timesteps, idx_com_rpy+2)-controller_data(Timesteps, idx_ref_com_rpy+2)))];
    legend(txt1,'ref','soln', 'location', 'northwest');

    subplot(2,3,4); hold on;
    plot(Time, estimated_data(Timesteps, idx_com_omega), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_com_omega), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_com_omega), 'b.-');
    hold off; grid minor;
    xlabel('Time (s)'); ylabel('\omega_X (rad / s)'); title('Body \omega_X');
    txt1 = ['estimated, RMSE: ' num2str( rms(estimated_data(Timesteps, idx_com_omega)-controller_data(Timesteps, idx_ref_com_omega)))];
    legend(txt1,'ref','soln', 'location', 'northwest');

    subplot(2,3,5); hold on;
    plot(Time, estimated_data(Timesteps, idx_com_omega + 1), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_com_omega + 1), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_com_omega + 1), 'b.-');
    hold off; grid minor;
    xlabel('Time (s)'); ylabel('\omega_Y (rad / s)'); title('Body \omega_Y');
    txt1 = ['estimated, RMSE: ' num2str( rms(estimated_data(Timesteps, idx_com_omega+1)-controller_data(Timesteps, idx_ref_com_omega+1)))];
    legend(txt1,'ref','soln', 'location', 'northwest');

    subplot(2,3,6); hold on;
    plot(Time, estimated_data(Timesteps, idx_com_omega + 2), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_com_omega + 2), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_com_omega + 2), 'b.-');
    hold off; grid minor;
    xlabel('Time (s)'); ylabel('\omega_Z (rad / s)'); title('Body \omega_Z');    
    txt1 = ['estimated, RMSE: ' num2str( rms(estimated_data(Timesteps, idx_com_omega+2)-controller_data(Timesteps, idx_ref_com_omega+2)))];
    legend(txt1,'ref','soln', 'location', 'northwest');
end


%% com pos, vel and acc
if com_pos_flag
    fig_num = fig_num + 1;
    figure(fig_num);

    subplot(3,3,1); 
    plot(Time, estimated_data(Timesteps, idx_com_pos), "k-" ); hold on;
    plot(Time, controller_data(Timesteps, idx_ref_com_pos), "r--" );
    %plot(Time, controller_data(Timesteps, idx_soln_com_pos), "b.-" ); hold off;
    grid minor; 
    xlabel('Time (s)'); ylabel('X Pos (m)'); title('Body X Pos');
    txt1 = ['estimated, RMSE: ' num2str( rms(estimated_data(Timesteps, idx_com_pos)-controller_data(Timesteps, idx_ref_com_pos)))];
    legend(txt1,'ref','soln', 'location', 'northwest');
    
    subplot(3,3,2); hold on;
    plot(Time, estimated_data(Timesteps, idx_com_pos + 1), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_com_pos + 1), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_com_pos + 1), 'b.-');
    hold off; grid minor;
    xlabel('Time (s)'); ylabel('Y Pos (m)'); title('Body Y Pos');
    txt1 = ['estimated, RMSE: ' num2str( rms(estimated_data(Timesteps, idx_com_pos+1)-controller_data(Timesteps, idx_ref_com_pos+1)))];
    legend(txt1,'ref','soln', 'location', 'northwest');

    subplot(3,3,3); hold on;
    plot(Time, estimated_data(Timesteps, idx_com_pos + 2), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_com_pos + 2), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_com_pos + 2), 'b.-');
    hold off; grid minor;
    xlabel('Time (s)'); ylabel('Z Pos (m)'); title('Body Z Pos');
    txt1 = ['estimated, RMSE: ' num2str( rms(estimated_data(Timesteps, idx_com_pos+2)-controller_data(Timesteps, idx_ref_com_pos+2)))];
    legend(txt1,'ref','soln', 'location', 'northwest');

    subplot(3,3,4); hold on;
    plot(Time, estimated_data(Timesteps, idx_com_vel), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_com_vel), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_com_vel), 'b.-');
    hold off; grid minor;
    xlabel('Time (s)'); ylabel('X Vel (m/s)'); title('Body X Vel');
    txt1 = ['estimated, RMSE: ' num2str( rms(estimated_data(Timesteps, idx_com_vel)-controller_data(Timesteps, idx_ref_com_vel)))];
    legend(txt1,'ref','soln', 'location', 'northwest');

    subplot(3,3,5); hold on;
    plot(Time, estimated_data(Timesteps, idx_com_vel + 1), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_com_vel + 1), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_com_vel + 1), 'b.-');
    hold off; grid minor;
    xlabel('Time (s)'); ylabel('Y Vel (m/s)'); title('Body Y Vel');
    txt1 = ['estimated, RMSE: ' num2str( rms(estimated_data(Timesteps, idx_com_vel+1)-controller_data(Timesteps, idx_ref_com_vel+1)))];
    legend(txt1,'ref','soln', 'location', 'northwest');

    subplot(3,3,6); hold on;
    plot(Time, estimated_data(Timesteps, idx_com_vel + 2), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_com_vel + 2), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_com_vel + 2), 'b.-');
    hold off; grid minor;
    xlabel('Time (s)'); ylabel('Z Vel (m/s)'); title('Body Z Vel');
    txt1 = ['estimated, RMSE: ' num2str( rms(estimated_data(Timesteps, idx_com_vel+2)-controller_data(Timesteps, idx_ref_com_vel+2)))];
    legend(txt1,'ref','soln', 'location', 'northwest');

    subplot(3,3,7); hold on;
    plot(Time, estimated_data(Timesteps, idx_com_acc), 'k-');
    hold off; grid minor; legend('estimated', 'location', 'northwest');
    xlabel('Time (s)'); ylabel('X Acc (m/s^{2})'); title('Body X Acc');  

    subplot(3,3,8); hold on;
    plot(Time, estimated_data(Timesteps, idx_com_acc + 1), 'k-');
    hold off; grid minor; legend('estimated', 'location', 'northwest');
    xlabel('Time (s)'); ylabel('Y Acc (m/s^{2})'); title('Body Y Acc');  

    subplot(3,3,9); hold on;
    plot(Time, estimated_data(Timesteps, idx_com_acc + 2), 'k-');
    hold off; grid minor; legend('estimated', 'location', 'northwest');
    xlabel('Time (s)'); ylabel('Z Acc (m/s^{2})'); title('Body Z Acc');  
end


%% Left foot rpy and omega
if foot_left_rpy_flag
    fig_num = fig_num + 1;
    figure(fig_num);

    subplot(2,3,1); 
    plot(Time, estimated_data(Timesteps, idx_foot_left_rpy), "k-" ); hold on;
    plot(Time, controller_data(Timesteps, idx_ref_foot_left_rpy), "r--" );
    %plot(Time, controller_data(Timesteps, idx_soln_foot_left_rpy), "b.-" ); hold off;
    grid minor; legend('estimated','ref','soln', 'location', 'southwest');
    xlabel('Time (s)'); ylabel('Roll (rad)'); title('Left Foot Roll');
    
    subplot(2,3,2); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_left_rpy + 1), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_left_rpy + 1), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_foot_left_rpy + 1), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'northwest');
    xlabel('Time (s)'); ylabel('Pitch (rad)'); title('Left Foot Pitch');

    subplot(2,3,3); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_left_rpy + 2), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_left_rpy + 2), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_foot_left_rpy + 2), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'west');
    xlabel('Time (s)'); ylabel('Yaw (rad)'); title('Left Foot Yaw');

    subplot(2,3,4); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_left_omega), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_left_omega), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_foot_left_omega), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'northwest');
    xlabel('Time (s)'); ylabel('\omega_x (rad/s)'); title('Left Foot \omega_x');

    subplot(2,3,5); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_left_omega + 1), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_left_omega + 1), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_foot_left_omega + 1), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'southwest');
    xlabel('Time (s)'); ylabel('\omega_y (rad/s)'); title('Left Foot \omega_y');

    subplot(2,3,6); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_left_omega + 2), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_left_omega + 2), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_foot_left_omega + 2), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'southwest');
    xlabel('Time (s)'); ylabel('\omega_z (rad/s)'); title('Left Foot \omega_z');
end

%% Left foot pos and vel
if foot_left_pos_flag
    fig_num = fig_num + 1;
    figure(fig_num);

    subplot(2,3,1); 
    plot(Time, estimated_data(Timesteps, idx_foot_left_pos), "k-" ); hold on;
    plot(Time, controller_data(Timesteps, idx_ref_foot_left_pos), "r--" );
    plot(Time, controller_data(Timesteps, idx_soln_foot_left_pos), "b.-" ); hold off;
    grid minor; legend('estimated','ref','soln', 'location', 'southwest');
    xlabel('Time (s)'); ylabel('X Pos (m)'); title('Left Foot X Pos');
    
    subplot(2,3,2); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_left_pos + 1), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_left_pos + 1), 'r--');
    plot(Time, controller_data(Timesteps, idx_soln_foot_left_pos + 1), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'northwest');
    xlabel('Time (s)'); ylabel('Y Pos (m)'); title('Left Foot Y Pos');
    

    subplot(2,3,3); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_left_pos + 2), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_left_pos + 2), 'r--');
    plot(Time, controller_data(Timesteps, idx_soln_foot_left_pos + 2), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'west');
    xlabel('Time (s)'); ylabel('Z Pos (m)'); title('Left Foot Z Pos');
    

    subplot(2,3,4); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_left_vel), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_left_vel), 'r--');
    plot(Time, controller_data(Timesteps, idx_soln_foot_left_vel), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'northwest');
    xlabel('Time (s)'); ylabel('X Vel (m/s)'); title('Left Foot X Vel');

    subplot(2,3,5); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_left_vel + 1), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_left_vel + 1), 'r--');
    plot(Time, controller_data(Timesteps, idx_soln_foot_left_vel + 1), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'southwest');
    xlabel('Time (s)'); ylabel('Y Vel (m/s)'); title('Left Foot Y Vel');

    subplot(2,3,6); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_left_vel + 2), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_left_vel + 2), 'r--');
    plot(Time, controller_data(Timesteps, idx_soln_foot_left_vel + 2), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'southwest');
    xlabel('Time (s)'); ylabel('Z Vel (m/s)'); title('Left Foot Z Vel');
end

%% Right foot rpy and omega
if foot_right_rpy_flag
    fig_num = fig_num + 1;
    figure(fig_num);

    subplot(2,3,1); 
    plot(Time, estimated_data(Timesteps, idx_foot_right_rpy), "k-" ); hold on;
    plot(Time, controller_data(Timesteps, idx_ref_foot_right_rpy), "r--" );
    %plot(Time, controller_data(Timesteps, idx_soln_foot_right_rpy), "b.-" ); hold off;
    grid minor; legend('estimated','ref','soln', 'location', 'southwest');
    xlabel('Time (s)'); ylabel('Roll (rad)'); title('right Foot Roll');
    
    subplot(2,3,2); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_right_rpy + 1), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_right_rpy + 1), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_foot_right_rpy + 1), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'northwest');
    xlabel('Time (s)'); ylabel('Pitch (rad)'); title('right Foot Pitch');

    subplot(2,3,3); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_right_rpy + 2), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_right_rpy + 2), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_foot_right_rpy + 2), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'west');
    xlabel('Time (s)'); ylabel('Yaw (rad)'); title('right Foot Yaw');

    subplot(2,3,4); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_right_omega), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_right_omega), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_foot_right_omega), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'northwest');
    xlabel('Time (s)'); ylabel('\omega_x (rad/s)'); title('right Foot \omega_x');

    subplot(2,3,5); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_right_omega + 1), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_right_omega + 1), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_foot_right_omega + 1), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'southwest');
    xlabel('Time (s)'); ylabel('\omega_y (rad/s)'); title('right Foot \omega_y');

    subplot(2,3,6); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_right_omega + 2), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_right_omega + 2), 'r--');
    %plot(Time, controller_data(Timesteps, idx_soln_foot_right_omega + 2), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'southwest');
    xlabel('Time (s)'); ylabel('\omega_z (rad/s)'); title('right Foot \omega_z');
end

%% Right foot pos and vel
if foot_right_pos_flag
    fig_num = fig_num + 1;
    figure(fig_num);

    subplot(2,3,1); 
    plot(Time, estimated_data(Timesteps, idx_foot_right_pos), "k-" ); hold on;
    plot(Time, controller_data(Timesteps, idx_ref_foot_right_pos), "r--" );
    plot(Time, controller_data(Timesteps, idx_soln_foot_right_pos), "b.-" ); hold off;
    grid minor; legend('estimated','ref','soln', 'location', 'northwest');
    xlabel('Time (s)'); ylabel('X Pos (m)'); title('Right Foot X Pos');
    
    subplot(2,3,2); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_right_pos + 1), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_right_pos + 1), 'r--');
    plot(Time, controller_data(Timesteps, idx_soln_foot_right_pos + 1), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'southwest');
    xlabel('Time (s)'); ylabel('Y Pos (m)'); title('Right Foot Y Pos');

    subplot(2,3,3); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_right_pos + 2), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_right_pos + 2), 'r--');
    plot(Time, controller_data(Timesteps, idx_soln_foot_right_pos + 2), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'west');
    xlabel('Time (s)'); ylabel('Z Pos (m)'); title('Right Foot Z Pos');

    subplot(2,3,4); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_right_vel), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_right_vel), 'r--');
    plot(Time, controller_data(Timesteps, idx_soln_foot_right_vel), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'southwest');
    xlabel('Time (s)'); ylabel('X Vel (m/s)'); title('Right Foot X Vel');

    subplot(2,3,5); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_right_vel + 1), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_right_vel + 1), 'r--');
    plot(Time, controller_data(Timesteps, idx_soln_foot_right_vel + 1), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'southwest');
    xlabel('Time (s)'); ylabel('Y Vel (m/s)'); title('Right Foot Y Vel');

    subplot(2,3,6); hold on;
    plot(Time, estimated_data(Timesteps, idx_foot_right_vel + 2), 'k-');
    plot(Time, controller_data(Timesteps, idx_ref_foot_right_vel + 2), 'r--');
    plot(Time, controller_data(Timesteps, idx_soln_foot_right_vel + 2), 'b.-');
    hold off; grid minor; legend('estimated','ref','soln', 'location', 'southwest');
    xlabel('Time (s)'); ylabel('Z Vel (m/s)'); title('Right Foot Z Vel');
end

%% Gyroscope Data and Bias
if gyro_flag
    fig_num = fig_num + 1;
    figure(fig_num);

    subplot(2,3,1); 
    plot(Time, sensor_data(Timesteps, idx_imu), "k-" ); grid minor;
    xlabel('Time (s)'); ylabel('\omega_X (rad/s)'); title('Gyro X');
    
    subplot(2,3,2);
    plot(Time, sensor_data(Timesteps, idx_imu + 1), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('\omega_Y (rad/s)'); title('Gyro Y');

    subplot(2,3,3);
    plot(Time, sensor_data(Timesteps, idx_imu + 2), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('\omega_Z (rad/s)'); title('Gyro Z');

    subplot(2,3,4);
    plot(Time, sensor_data(Timesteps, idx_imu_bias), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('\omega_X Bias (rad/s)'); title('Gyro X Bias');

    subplot(2,3,5);
    plot(Time, estimated_data(Timesteps, idx_imu_bias + 1), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('\omega_Y Bias (rad/s)'); title('Gyro Y Bias');

    subplot(2,3,6);
    plot(Time, estimated_data(Timesteps, idx_imu_bias + 2), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('\omega_Z Bias (rad/s)'); title('Gyro Z Bias');
end

%% Accelerometer Data and Bias
if acc_flag
    fig_num = fig_num + 1;
    figure(fig_num);

    subplot(2,3,1); 
    plot(Time, sensor_data(Timesteps, idx_imu + 3), "k-" ); grid minor;
    xlabel('Time (s)'); ylabel('X Acc (m/s^{2})'); title('Acc X');
    
    subplot(2,3,2);
    plot(Time, sensor_data(Timesteps, idx_imu + 4), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Y Acc (m/s^{2})'); title('Acc Y');

    subplot(2,3,3);
    plot(Time, sensor_data(Timesteps, idx_imu + 5), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Z Acc (m/s^{2})'); title('Acc Z');

    subplot(2,3,4);
    plot(Time, sensor_data(Timesteps, idx_imu_bias + 3), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('X Acc Bias (m/s^{2})'); title('Acc X Bias');

    subplot(2,3,5);
    plot(Time, estimated_data(Timesteps, idx_imu_bias + 4), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Y Acc Bias (m/s^{2})'); title('Acc Y Bias');

    subplot(2,3,6);
    plot(Time, estimated_data(Timesteps, idx_imu_bias + 5), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Z Acc Bias (m/s^{2})'); title('Acc Z Bias');
end

%% Left Joint Data

if left_joint_flag
    fig_num = fig_num + 1;
    figure(fig_num);

    subplot(3,5,1); 
    plot(Time, sensor_data(Timesteps, idx_joint_pos), "k-" ); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Left Joint 1 Pos');
    
    subplot(3,5,2);
    plot(Time, sensor_data(Timesteps, idx_joint_pos + 1), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Left Joint 2 Pos');

    subplot(3,5,3);
    plot(Time, sensor_data(Timesteps, idx_joint_pos + 2), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Left Joint 3 Pos');

    subplot(3,5,4);
    plot(Time, sensor_data(Timesteps, idx_joint_pos + 3), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Left Joint 4 Pos');

    subplot(3,5,5);
    plot(Time, sensor_data(Timesteps, idx_joint_pos + 4), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Left Joint 5 Pos');

    subplot(3,5,6);
    plot(Time, sensor_data(Timesteps, idx_joint_vel), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Left Joint 1 Vel');

    subplot(3,5,7);
    plot(Time, sensor_data(Timesteps, idx_joint_vel + 1), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Left Joint 2 Vel');

    subplot(3,5,8);
    plot(Time, sensor_data(Timesteps, idx_joint_vel + 2), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Left Joint 3 Vel');

    subplot(3,5,9);
    plot(Time, sensor_data(Timesteps, idx_joint_vel + 3), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Left Joint 4 Vel');

    subplot(3,5,10);
    plot(Time, sensor_data(Timesteps, idx_joint_vel + 4), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Left Joint 5 Vel');

    subplot(3,5,11);
    plot(Time, sensor_data(Timesteps, idx_joint_tau), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Left Joint 1 Torque');

    subplot(3,5,12);
    plot(Time, sensor_data(Timesteps, idx_joint_tau + 1), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Left Joint 2 Torque');

    subplot(3,5,13);
    plot(Time, sensor_data(Timesteps, idx_joint_tau + 2), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Left Joint 3 Torque');

    subplot(3,5,14);
    plot(Time, sensor_data(Timesteps, idx_joint_tau + 3), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Left Joint 4 Torque');

    subplot(3,5,15);
    plot(Time, sensor_data(Timesteps, idx_joint_tau + 4), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Left Joint 5 Torque');
end

%% Right Joint data

if right_joint_flag
    fig_num = fig_num + 1;
    figure(fig_num);

    subplot(3,5,1); 
    plot(Time, sensor_data(Timesteps, idx_joint_pos + 5), "k-" ); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Right Joint 1 Pos');
    
    subplot(3,5,2);
    plot(Time, sensor_data(Timesteps, idx_joint_pos + 6), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Right Joint 2 Pos');

    subplot(3,5,3);
    plot(Time, sensor_data(Timesteps, idx_joint_pos + 7), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Right Joint 3 Pos');

    subplot(3,5,4);
    plot(Time, sensor_data(Timesteps, idx_joint_pos + 8), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Right Joint 4 Pos');

    subplot(3,5,5);
    plot(Time, sensor_data(Timesteps, idx_joint_pos + 9), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Right Joint 5 Pos');

    subplot(3,5,6);
    plot(Time, sensor_data(Timesteps, idx_joint_vel + 5), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Right Joint 1 Vel');

    subplot(3,5,7);
    plot(Time, sensor_data(Timesteps, idx_joint_vel + 6), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Right Joint 2 Vel');

    subplot(3,5,8);
    plot(Time, sensor_data(Timesteps, idx_joint_vel + 7), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Right Joint 3 Vel');

    subplot(3,5,9);
    plot(Time, sensor_data(Timesteps, idx_joint_vel + 8), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Right Joint 4 Vel');

    subplot(3,5,10);
    plot(Time, sensor_data(Timesteps, idx_joint_vel + 9), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Right Joint 5 Vel');

    subplot(3,5,11);
    plot(Time, sensor_data(Timesteps, idx_joint_tau + 5), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Right Joint 1 Torque');

    subplot(3,5,12);
    plot(Time, sensor_data(Timesteps, idx_joint_tau + 6), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Right Joint 2 Torque');

    subplot(3,5,13);
    plot(Time, sensor_data(Timesteps, idx_joint_tau + 7), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Right Joint 3 Torque');

    subplot(3,5,14);
    plot(Time, sensor_data(Timesteps, idx_joint_tau + 8), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Right Joint 4 Torque');

    subplot(3,5,15);
    plot(Time, sensor_data(Timesteps, idx_joint_tau + 9), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Right Joint 5 Torque');
end

%% GRF Left Data
if GRF_left_flag
    fig_num = fig_num + 1;
    figure(fig_num);

    subplot(1,3,1); 
    plot(Time, controller_data(Timesteps, idx_GRF_left), "k-" ); grid minor;
    xlabel('Time (s)'); ylabel('GRF (N)'); title('Left GRF X');
    
    subplot(1,3,2);
    plot(Time, controller_data(Timesteps, idx_GRF_left + 1), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('GRF (N)'); title('Left GRF Y');

    subplot(1,3,3);
    plot(Time, controller_data(Timesteps, idx_GRF_left + 2), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('GRF (N)'); title('Left GRF Z');
end

%% GRF Right Data
if GRF_right_flag
    fig_num = fig_num + 1;
    figure(fig_num);

    subplot(1,3,1); 
    plot(Time, controller_data(Timesteps, idx_GRF_right), "k-" ); grid minor;
    xlabel('Time (s)'); ylabel('GRF (N)'); title('Right GRF X');
    
    subplot(1,3,2);
    plot(Time, controller_data(Timesteps, idx_GRF_right + 1), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('GRF (N)'); title('Right GRF Y');

    subplot(1,3,3);
    plot(Time, controller_data(Timesteps, idx_GRF_right + 2), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('GRF (N)'); title('Right GRF Z');
end

%% GRM Left Data
if GRM_left_flag
    fig_num = fig_num + 1;
    figure(fig_num);

    subplot(1,3,1); 
    plot(Time, controller_data(Timesteps, idx_GRM_left), "k-" ); grid minor;
    xlabel('Time (s)'); ylabel('GRM (kg*m/s)'); title('Left GRM X');
    
    subplot(1,3,2);
    plot(Time, controller_data(Timesteps, idx_GRM_left + 1), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('GRM (kg*m/s)'); title('Left GRM Y');

    subplot(1,3,3);
    plot(Time, controller_data(Timesteps, idx_GRM_left + 2), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('GRM (kg*m/s)'); title('Left GRM Z');
end

%% GRM Right Data
if GRM_right_flag
    fig_num = fig_num + 1;
    figure(fig_num);

    subplot(1,3,1); 
    plot(Time, controller_data(Timesteps, idx_GRM_right), "k-" ); grid minor;
    xlabel('Time (s)'); ylabel('GRM (kg*m/s)'); title('Right GRM X');
    
    subplot(1,3,2);
    plot(Time, controller_data(Timesteps, idx_GRM_right + 1), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('GRM (kg*m/s)'); title('Right GRM Y');

    subplot(1,3,3);
    plot(Time, controller_data(Timesteps, idx_GRM_right + 2), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('GRM (kg*m/s)'); title('Right GRM Z');
end

%% Joint CMD Data

if joint_cmd_flag
    fig_num = fig_num + 1;
    figure(fig_num);

    subplot(3,5,1); 
    plot(Time, controller_data(Timesteps, idx_joint_cmd_pos), "k-" ); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Left Joint 1 CMD Pos');
    
    subplot(3,5,2);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_pos + 1), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Left Joint 2 CMD Pos');

    subplot(3,5,3);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_pos + 2), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Left Joint 3 CMD Pos');

    subplot(3,5,4);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_pos + 3), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Left Joint 4 CMD Pos');

    subplot(3,5,5);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_pos + 4), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Left Joint 5 CMD Pos');

    subplot(3,5,6);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_vel), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Left Joint 1 CMD Vel');

    subplot(3,5,7);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_vel + 1), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Left Joint 2 CMD Vel');

    subplot(3,5,8);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_vel + 2), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Left Joint 3 CMD Vel');

    subplot(3,5,9);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_vel + 3), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Left Joint 4 CMD Vel');

    subplot(3,5,10);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_vel + 4), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Left Joint 5 CMD Vel');

    subplot(3,5,11);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_tau), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Left Joint 1 CMD Torque');

    subplot(3,5,12);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_tau + 1), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Left Joint 2 CMD Torque');

    subplot(3,5,13);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_tau + 2), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Left Joint 3 CMD Torque');

    subplot(3,5,14);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_tau + 3), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Left Joint 4  CMD Torque');

    subplot(3,5,15);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_tau + 4), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Left Joint 5 CMD Torque');

    fig_num = fig_num + 1;
    figure(fig_num);

    subplot(3,5,1); 
    plot(Time, controller_data(Timesteps, idx_joint_cmd_pos + 5), "k-" ); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Right Joint 1 CMD Pos');
    
    subplot(3,5,2);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_pos + 6), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Right Joint 2 CMD Pos');

    subplot(3,5,3);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_pos + 7), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Right Joint 3 CMD Pos');

    subplot(3,5,4);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_pos + 8), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Right Joint 4 CMD Pos');

    subplot(3,5,5);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_pos + 9), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Pos (m)'); title('Right Joint 5 CMD Pos');

    subplot(3,5,6);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_vel + 5), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Right Joint 1 CMD Vel');

    subplot(3,5,7);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_vel + 6), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Right Joint 2 CMD Vel');

    subplot(3,5,8);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_vel + 7), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Right Joint 3 CMD Vel');

    subplot(3,5,9);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_vel + 8), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Right Joint 4 CMD Vel');

    subplot(3,5,10);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_vel + 9), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Vel (m/s)'); title('Right Joint 5 CMD Vel');

    subplot(3,5,11);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_tau + 5), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Right Joint 1 CMD Torque');

    subplot(3,5,12);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_tau + 6), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Right Joint 2 CMD Torque');

    subplot(3,5,13);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_tau + 7), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Right Joint 3 CMD Torque');

    subplot(3,5,14);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_tau + 8), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Right Joint 4  CMD Torque');

    subplot(3,5,15);
    plot(Time, controller_data(Timesteps, idx_joint_cmd_tau + 9), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Right Joint 5 CMD Torque');
end

%% Control Solver Data
if ctrl_solver_flag
    fig_num = fig_num + 1;
    figure(fig_num);

    subplot(1,3,1); 
    plot(Time, controller_data(Timesteps, idx_ctrl_solver_cost), "k-" ); grid minor;
    xlabel('Time (s)'); ylabel('Cost'); title('Solver Cost');
    
    subplot(1,3,2);
    plot(Time, controller_data(Timesteps, idx_ctrl_solver_time), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Time (s)'); title('Solver Time');

    subplot(1,3,3);
    plot(Time, controller_data(Timesteps, idx_ctrl_solver_iter), 'k-'); grid minor;
    xlabel('Time (s)'); ylabel('Iterations'); title('Solver Iterations');
end
%% Future Code

%     max_y = max(controller_data(Timesteps, idx_com_rpy)); min_y = min(controller_data(Timesteps, idx_com_rpy));
%     if max_y > 0 && min_y > 0
%         bar_l_plan = bar(contact_plan(1,1,:),0.1*max_y*bar_contact_plan(1,:), 1);       set(bar_l_plan,'FaceAlpha',0.25, 'FaceColor',[1 0 0], 'Linestyle','none');
%         bar_l_est = bar(contact_est(1,:),1.1*max_y*bar_contact_est(1,:), 1);                set(bar_l_est,'FaceAlpha',0.5, 'FaceColor',[1 0 0], 'Linestyle','none');
%         bar_r_plan = bar(contact_plan(2,1,:),0.1*max_y*bar_contact_plan(1,:), 1);       set(bar_r_plan,'FaceAlpha',0.25, 'FaceColor',[0 0 1], 'Linestyle','none');
%         bar_r_est = bar(contact_est(1,:),1.1*max_y*bar_contact_est(1,:), 1);                set(bar_r_est,'FaceAlpha',0.25, 'FaceColor',[0 0 1], 'Linestyle','none');
%     elseif max_y > 0 && min_y < 0  
%         bar_l_plan_pos = bar(contact_plan(1,1,:),0.1*max_y*bar_contact_plan(1,:), 1);   set(bar_l_plan_pos,'FaceAlpha',0.25, 'FaceColor',[1 0 0], 'Linestyle','none');
%         bar_l_est_pos = bar(contact_est(1,:),1.1*max_y*bar_contact_est(1,:), 1);            set(bar_l_est_pos,'FaceAlpha',0.25, 'FaceColor',[1 0 0], 'Linestyle','none');
%         bar_r_plan_pos = bar(contact_plan(2,1,:),0.1*max_y*bar_contact_plan(1,:), 1);   set(bar_r_plan_pos,'FaceAlpha',0.25, 'FaceColor',[0 0 1], 'Linestyle','none');
%         bar_r_est_pos = bar(contact_est(1,:),1.1*max_y*bar_contact_est(1,:), 1);            set(bar_r_est_pos,'FaceAlpha',0.25, 'FaceColor',[0 0 1], 'Linestyle','none');
%         bar_l_plan_neg = bar(contact_plan(1,1,:),0.1*min_y*bar_contact_plan(1,:), 1);  set(bar_l_plan_neg,'FaceAlpha',0.25, 'FaceColor',[1 0 0], 'Linestyle','none');
%         bar_l_est_neg = bar(contact_est(1,:),1.1*min_y*bar_contact_est(1,:), 1);           set(bar_l_est_neg,'FaceAlpha',0.25, 'FaceColor',[1 0 0], 'Linestyle','none');
%         bar_r_plan_neg = bar(contact_plan(2,1,:),0.1*min_y*bar_contact_plan(1,:), 1);  set(bar_r_plan_neg,'FaceAlpha',0.25, 'FaceColor',[0 0 1], 'Linestyle','none');
%         bar_r_est_neg = bar(contact_est(1,:),1.1*min_y*bar_contact_est(1,:), 1);           set(bar_r_est_neg,'FaceAlpha',0.25, 'FaceColor',[0 0 1], 'Linestyle','none');
%     else % both are smaller than 0
%         bar_l_plan = bar(contact_plan(1,1,:),0.1*min_y*bar_contact_plan(1,:), 1);      set(bar_l_plan,'FaceAlpha',0.25, 'FaceColor',[1 0 0], 'Linestyle','none');
%         bar_l_est = bar(contact_est(1,:),1.1*min_y*bar_contact_est(1,:), 1);               set(bar_l_est,'FaceAlpha',0.25, 'FaceColor',[1 0 0], 'Linestyle','none');
%         bar_r_plan = bar(contact_plan(2,1,:),0.1*min_y*bar_contact_plan(1,:), 1);      set(bar_r_plan,'FaceAlpha',0.25, 'FaceColor',[0 0 1], 'Linestyle','none');
%         bar_r_est = bar(contact_est(1,:),1.1*min_y*bar_contact_est(1,:), 1);               set(bar_r_est,'FaceAlpha',0.25, 'FaceColor',[0 0 1], 'Linestyle','none');
%     end

