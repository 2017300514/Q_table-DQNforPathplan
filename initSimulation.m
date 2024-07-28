
clc, clear all, close all, warning('off');


%% ----- Globale variabler

global mode;        
global figure1;
global figure2;
figure1 = true;     
figure2 = true;
mode = '';


%% ----- Initialize paramters for simulations

i = 1;              % Total trials with Q-table method
j = 1;              % Total trials with Neural Network method
n1 = 1;             
n2 = 1;             
x1 = 0;             
x2 = 0;             
y1 = 0;             
y2 = 0;             
y3 = 0;             
y4 = 0; 

steps1 = 0;
steps2 = 0;

n_steps1 = 0;
n_steps2 = 0;

tot_steps1 = 0;
tot_steps2 = 0;

percent1 = 0;
percent2 = 0;

tot_percent1 = 0;
tot_percent2 = 0;
n_crash1 = 0;       % total number of crash with Q-table
n_crash2 = 0;       % total number of crash with Nueral-Network

n_arrive1 = 0;      % total number of arrive with Q-table
n_arrive2 = 0;      % total number of arrive with Nueral-Network



reward1 = 0;        % reward of the Q-tabel
reward2 = 0;        % reward of the Nueral-Network

total_reward1 = 0;  % total reward of the Q-table
total_reward2 = 0;  % total reward of the Nueral-Network
cost_fun = 0;       %  待删除?
% Select between static and dynamic obstacles
% dynamic obstacles = 'Dynamic', static obstacles = 'Static'
% stadium_option1 = 'Static';
stadium_option1 = 'Static';
% stadium_option2 = 'Static';
stadium_option2 = 'Static';
% Properties of the vehicle. scale: 1:10 cm

radius = 2;
sensor_lengde = 10;
tot_rot2 = 0;
tot_rot1 = 0;
figure_num=1;     % 画图的序号

gettable = 0;
getdqn = 0 ;     %辅助记录完成路径的步长 的变量
record_table = zeros(500,3);     % 记录完成路径的步长信息 ，500 是 数据容量  3 包括 图编号 预定路径编号 3 4 5 6 8 10 步长
record_dqn = zeros(500,3);
%% ----- Initialize Q-learing with Q-table 
state_space = createStateSpace(stadium_option1);    % state space
actions = actionList();                             % action list

%Find the size of the state space and the length of the action list
n_states = size(state_space,1);
n_actions = length(actions);

% Initialize Q-table and the model
Q_table = createQTable(n_states, n_actions);      %Q-table


%% ----- Initialize Q-learning with Neural Network

% Neural Network
% input_layer_size = 5;
% hidden_layer_size = 15;
input_layer_size = 16;   % sensor,ang,dist_a_obj  sensor 14个数据 ang 1个 dist_a_obj 1个
hidden_layer_size = 300;
output_layer_size = 3;
pool_top = 10000;
learn_pollX = zeros(2*input_layer_size+3,pool_top);      % pool(:,pool_count) = [sensor,ang,dist_a_obj,temp_sensor,pre_ang,last_a_obj,reward,a];
learn_pollY = xlsread("dqn_pool23.xlsx");
learn_poll_numY = size(learn_pollY,2)+1;
learn_poll_numX = 1;
write_check = 1;

%DQN 学习池子

initial_w1 = randInitializeWeights(input_layer_size, hidden_layer_size);
initial_w2 = randInitializeWeights(hidden_layer_size, output_layer_size);
nn_params = [initial_w1(:); initial_w2(:)];
target_params = nn_params;
%% ----- Initaliseringer av parametrer for Q-tabell og Neural Network metoden

%Initalisreringer av forskjellige parametrer
max_trials = 700;     % Max Trials
max_steps = 1000;        % Max steps
alpha1  = 0.5;          % Learning rate
alpha2  = 0.03;
gamma   = 0.9;          % Discount factor
epsilon = 0.95;         % Chance for random action
T1 = 24;                % The Paramater of the explore function with blotzmanns distribution
T2 = 24;

arrive_rate = zeros(1,max_trials/100);
%% ----- Initaliseringer for GUI
   %GUI 图形用户界面
%Plottets st鴕relse
xlimit = [0, 100];
ylimit = [0, 100];

% GUI
[h_plot1, h_plot2, h_text1, h_text2, h_button1, h_button2] = createSimulation(xlimit, ylimit, max_trials, max_steps,...
    alpha1, alpha2, gamma, epsilon, T1);
% Simulation Environments
subplot(h_plot1(1));
[h_poly1, h_circ1,init_plot_list1,guide_list1,guide_num1,theta_list1,theta_num1] = createStadium(stadium_option1);


choice1_1 = floor(3*rand())+1;   %  随机数乘三向下取整 均匀概率取 1,2,3

guide_list_now1 = guide_list1(:,2*choice1_1-1:2*choice1_1);

init_plot1 = init_plot_list1(choice1_1,:);

choice1_2 = floor(2*rand());

goal_list1 = guide_list_now1(guide_num1*choice1_2+1:guide_num1*choice1_2+guide_num1,:);

choice_for_theta1 = floor(theta_num1*rand())+1;   %随机数 生成1 到 theta_num  抽取一个初始角度

theta1 = theta_list1(choice_for_theta1);

h_car1 = createCarCircle(radius, sensor_lengde, 'b', init_plot1, theta1);

tot_rot1 = tot_rot1 + theta1;
%% 下面是 DQN方法的初始化


subplot(h_plot2(1));
[h_poly2, h_circ2,init_plot_list2,guide_list2,guide_num2,theta_list2,theta_num2] = createStadium(stadium_option2);

choice2_1 = floor(3*rand())+1;   %  随机数乘三向下取整 均匀概率取 1,2,3

guide_list_now2 = guide_list2(:,2*choice2_1-1:2*choice2_1);

init_plot2 = init_plot_list2(choice2_1,:);  

choice2_2 = floor(2*rand());

goal_list2 = guide_list_now2(guide_num2*choice2_2+1:guide_num2*choice2_2+guide_num2,:);

choice_for_theta2 = floor(theta_num2*rand())+1;   %随机数 生成1 到 theta_num  抽取一个初始角度

theta2 = theta_list2(choice_for_theta2);

h_car2 = createCarCircle(radius, sensor_lengde, 'b', init_plot2, theta2);

tot_rot2 = tot_rot2 +theta2;