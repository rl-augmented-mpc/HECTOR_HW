function draw_robot(fig,robot_model)
%DRAW_ROBOT 이 함수의 요약 설명 위치
%   자세한 설명 위치
figure(fig)
clf;
front_calf_joint_position = robot_model.x + robot_model.R*[0.4 + robot_model.thigh_length * sin(robot_model.q(1));0;robot_model.thigh_length * cos(robot_model.q(1))];
rear_calf_joint_position = robot_model.x + robot_model.R*[-0.4 + robot_model.thigh_length * sin(robot_model.q(3));0;robot_model.thigh_length * cos(robot_model.q(3))];
front_thigh_joint_position = robot_model.x + robot_model.R*[0.4;0;0];
rear_thigh_joint_position = robot_model.x + robot_model.R*[-0.4;0;0];
front_foot_position = robot_model.x + robot_model.R*robot_model.x_foot(1:3);
rear_foot_position = robot_model.x + robot_model.R*robot_model.x_foot(4:6);

hold on
plot([front_thigh_joint_position(1),front_calf_joint_position(1)],[front_thigh_joint_position(3),front_calf_joint_position(3)],'k-', 'LineWidth',2);
plot([rear_thigh_joint_position(1),rear_calf_joint_position(1)],[rear_thigh_joint_position(3),rear_calf_joint_position(3)],'k-', 'LineWidth',2);
plot([front_calf_joint_position(1),front_foot_position(1)],[front_calf_joint_position(3),front_foot_position(3)],'k-', 'LineWidth',2);
plot([rear_calf_joint_position(1),rear_foot_position(1)],[rear_calf_joint_position(3),rear_foot_position(3)],'k-', 'LineWidth',2);

position = [robot_model.R*[-0.5;0;0.2]+robot_model.x,robot_model.R*[-0.5;0;-0.2]+robot_model.x,robot_model.R*[0.5;0;-0.2]+robot_model.x,robot_model.R*[0.5;0;0.2]+robot_model.x,robot_model.R*[-0.5;0;0.2]+robot_model.x];

for i=1:4
    plot([position(1,i),position(1,i+1)],[position(3,i),position(3,i+1)],'k-', 'LineWidth',2);
end
x=robot_model.x;
plot(x(1),x(3),'k*','LineWidth',2);


% axis([-3 3 -3 3])
axis equal
axis([-3 3 -3 3])
hold off
end

