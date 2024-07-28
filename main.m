
initSimulation;

%% ----- Main loop

while(true)
    
    pause(0.1);
    
    
    %% Reset 功能   for Q-tabel
    
    if( strcmp(mode,'Reset1') )
        
        mode = ''; 
        figure1 = true;
        i = 1;
        n1 = 1;
        x1 = 0;
        y1 = 0;
        y2 = 0;
        steps1 = 0;
        n_steps1 = 0;
        reward1 = 0;
        total_reward1 = 0;
        n_crash1 = 0;
        T1 = 24;
        
        Q_table = createQTable(n_states, n_actions);
        
        cla(h_plot1(2));
        cla(h_plot1(3));
    end
    
    %% upload 模块 用来代入数据
    if( strcmp(mode,'Upload1') )
        
        mode = '';
        
        if( exist('Q-Table_upload_static.mat', 'file') >= 1 || exist('Q-Table_upload_dynamic.mat', 'file') >= 1 )
            
            if(strcmp(stadium_option1,'Static') == 1), load('Q-Table_upload_static.mat');
                
            elseif(strcmp(stadium_option1,'Dynamic') == 1), load('Q-Table_upload_dynamic.mat');
                
            end
            
            subplot(h_plot1(2));
            plot(x1,y1);

            subplot(h_plot1(3));
            plot(x1,y2);

            i = i + 1;
            T1 = 1.0e-3;
            
        end
        
    end
    %% 储存模块
    if( strcmp(mode,'Save1') )
        
        mode = '';
        
        % Store pamaterers
        save('Q_Table_savefile.mat', 'Q_table', 'i', 'n1', 'x1', 'y1', 'y2', 'steps1', 'n_steps1',...
            'reward1', 'total_reward1', 'n_crash1','tot_steps1');
        
        % Store MATLAB figure
        savefig('Q_table_fig');
        
        % Store Q-table
        fil = fopen('Q-tabell&stateSpace.txt','w');
        fprintf(fil, '%25s%34s\n','Q-tabell', 'State Space');
        
        if( strcmp(stadium_option1,'Static') == 1)
            fprintf(fil, '%10s%10s%10s\t\t%5s%5s%5s%5s\n','Q1', 'Q2', 'Q3', 'k1', 'k2', 'k3', 'k4');
            fprintf(fil,'%10.2f%10.2f%10.2f\t\t%5.f%5.f%5.f%5.f\n', cat(2,Q_table,state_space)');
            
        elseif(strcmp(stadium_option1,'Dynamic') == 1)
            fprintf(fil, '%10s%10s%10s\t\t%5s%5s%5s%5s%5s%5s\n','Q1', 'Q2', 'Q3', 'k1', 'k2', 'k3', 'k4', 'k5', 'k6');
            fprintf(fil,'%10.2f%10.2f%10.2f\t\t%5.f%5.f%5.f%5.f%5.f%5.f\n', cat(2,Q_table,state_space)');
        end
        fclose(fil);
        
        set(h_button1(5), 'BackGroundColor', [0.4,0.4,0.4], 'Enable', 'off');
        
        break;
    end
    
    
    %% Reset for Neural Network 
    if( strcmp(mode,'Reset2') )
        
        mode = '';
        figure2 = true;
        j = 1;
        n2 = 1;
        x2 = 0;
        y3 = 0;
        y4 = 0;
        steps2 = 0;
        n_steps2 = 0;
        n_arrive2 = 0;
        reward2 = 0;
        total_reward2 = 0;
        n_crash2 = 0;
        T2 = 24;
        
        initial_w1 = randInitializeWeights(input_layer_size, hidden_layer_size);
        initial_w2 = randInitializeWeights(hidden_layer_size, output_layer_size);
        nn_params = [initial_w1(:); initial_w2(:)];
        
        cla(h_plot2(2));
        cla(h_plot2(3));
        
    end
    
    if( strcmp(mode,'Upload2') )
        
        mode = '';
        
        if( exist('NN_upload_static.mat', 'file') >= 1 || exist('NN_upload_dynamic.mat', 'file') >= 1 )
            
            if(strcmp(stadium_option2,'Static') == 1), load('NN_upload_static.mat');                
            elseif(strcmp(stadium_option2,'Dynamic') == 1), load('NN_upload_dynamic.mat');
            end
            
            load('NN_uploadfile.mat');
            
            subplot(h_plot2(2));
            plot(x2,y3);
            
            subplot(h_plot2(3));
            plot(x2,y4);
            
            j = j + 1;
            T2 = 1.0e-3;
            
        end
    end
    
    if( strcmp(mode,'Save2') )
        
        mode = '';
        
        weight1 = reshape(nn_params(1:hidden_layer_size * (input_layer_size + 1)), ...
            hidden_layer_size, (input_layer_size + 1));
        
        weight2 = reshape(nn_params((1 + (hidden_layer_size * (input_layer_size + 1))):end), ...
            output_layer_size, (hidden_layer_size + 1));
        
        %% Store paramters
        save('NN_savefile.mat', 'nn_params', 'weight1', 'weight2', 'j', 'n2', 'x2', 'y3', 'y4', 'steps2', 'n_steps2',...
            'reward2', 'total_reward2', 'n_crash2','tot_steps2','cost_fun');
        
        %% Store MATLAB figure
        savefig('nn_fig');
        
        %% Store weight matrices
        fil = fopen('nnWeights.txt','w');
        fprintf(fil, '\n%30s\n','Weight1');
        
        fprintf(fil, '%8s%8s%8s%8s%8s%8s\n','w0', 'w1', 'w2', 'w3', 'w4', 'w5');
        fprintf(fil,'%8.3f%8.3f%8.3f%8.3f%8.3f%8.3f\n', weight1');
        
        fprintf(fil, '\n\n%65s\n','Weight2');
        fprintf(fil, '%8s%8s%8s%8s%8s%8s%8s%8s%8s%8s%8s%8s%8s%8s%8s%8s\n',...
            'w0', 'w1', 'w2', 'w3', 'w4', 'w5', 'w6','w7', 'w8', 'w9', 'w10', 'w11', 'w12', 'w13', 'w14', 'w15');
        fprintf(fil, '%8.3f%8.3f%8.3f%8.3f%8.3f%8.3f%8.3f%8.3f%8.3f%8.3f%8.3f%8.3f%8.3f%8.3f%8.3f%8.3f\n', weight2');
        
        fclose(fil);
        
        clear('weight1', 'weight2');
        set(h_button2(5), 'BackGroundColor', [0.4,0.4,0.4], 'Enable', 'off');
        
    end
    
    
    %% Main loop for Q-table method     主循环
    if( strcmp(mode,'Start1') )
        
        mode = '';
        info_s1 = sprintf('%s%11d%21s%13d%22s%10.2f', 'Trials:', i-1, 'Steps:', steps1, 'Total reward:', reward1);    
        %存疑，info_s1是啥
        t1 = clock ;
        
        for i = i:max_trials
            
            figure(1);
            
            subplot(h_plot1(1));
            
            
            [Q_table, reward1,steps1,percent1, n_crash1, n_arrive1,info_s2,figure_num,learn_pollX,learn_poll_numX] = tableTrials(Q_table, alpha1, gamma, T1,stadium_option1, n_arrive1 ,tot_rot1,  ...
                                                                                 h_car1,h_circ1, h_poly1, state_space, actions, max_steps, n_crash1,  ...
                                                                                 info_s1, h_text1,init_plot1,goal_list1,guide_num1,figure_num,learn_pollX,learn_poll_numX);
                                                                                
            
            
            if (percent1 ==5 )
                
                if  gettable == 0  
                    
                    t2 = clock;
                    first_getTime = etime(t2,t1);
                end
                gettable =gettable+ 1;
                record_table(gettable,:) = [figure_num,(choice1_1+2)*(choice1_2+1),steps1];
            end                                                                 
            figure(1);           
            
            subplot(h_plot1(1));
                                                                             
            delete(h_car1);
            
            choice1_1 = floor(3*rand())+1;   %  随机数乘三向下取整 均匀概率取 1,2,3
            
            guide_list_now1 = guide_list1(:,2*choice1_1-1:2*choice1_1);
            
            init_plot1 = init_plot_list1(choice1_1,:);
            
            choice1_2 = floor(2*rand());
            
            goal_list1= guide_list_now1(guide_num1*choice1_2+1:guide_num1*choice1_2+guide_num1,:);
            
            choice_for_theta1 = floor(theta_num1*rand())+1;   %随机数 生成1 到 theta_num  抽取一个初始角度

            theta1 = theta_list1(choice_for_theta1);
            
            h_car1 = createCarCircle(radius, sensor_lengde, 'b',init_plot1, theta1);
            
            tot_rot1 = theta1;

            
           
            if(learn_poll_numX>=10000*write_check)
                
                filename = ['dqn_pool' num2str(write_check) '.xlsx'];
                xlswrite(filename , learn_pollX);
                write_check = write_check + 1;
            end
           
            if( strcmp(stadium_option2, 'Dynamic') == 1 )
                delete(h_circ1(4));
                delete(h_circ1(5));
                [X,Y] = circle([30, 50], 2, 100);
                cir1_2 = patch(X,Y,'y','EdgeColor', 'b');
                [X,Y] = circle([60, 60], 2, 100);
                cir1_3 = patch(X,Y,'y','EdgeColor', 'b');        %更新两个动态障碍物 
                h_circ1(4)= cir1_2;
                h_circ1(5)= cir1_3;
            end
            

            drawnow;                                         %更新图窗，处理回调
            
            if( strcmp(mode,'Pause1') ), break; end
            if(T1 > 1.0e-3), T1 = 0.95*T1; end
            
            
            info_s1 = sprintf('%s%11d%21s%13d%22s%10.2f', 'Trials:', i, 'Steps:', steps1, 'Total reward:', reward1);
            info_s = sprintf('%s\n%s', info_s1, info_s2);
            set(h_text1, 'String', info_s);
            
            n_steps1 = n_steps1 + steps1;
            tot_steps1 = tot_steps1 + steps1;
            tot_percent1 = tot_percent1 + percent1;            %累计十步的到达距离半分比
            total_reward1 = total_reward1 + reward1;
%%  处理输出数据图
            if (mod(i,100)== 0)
                act = i/100;
                arrive_rate(act) = n_arrive1/5 - sum(arrive_rate);              
            end
            if( mod(i,10) == 0)       % mod 求余数
                
                n1 = n1 + 1;
                x1(n1) = n1-1;
                
                y1(n1) = tot_percent1*2;   % =tot_percent1/50*100(%)
                
                y2(n1) = total_reward1/10;
                
                subplot(h_plot1(2));
                plot(x1,y1);
                xlabel('Trials x10');
                ylabel('Average percentage of distance each 10th trials');    
                
                subplot(h_plot1(3));
                plot(x1,y2);
%                 title('Total reward');
                xlabel('Trials x10');
                ylabel('Average reward of each 10th trials');
                
%                 n_steps1 = 0;
                total_reward1 = 0;
                tot_percent1 = 0;
                subplot(h_plot1(1));
                drawnow;
            end
 %%           
            if( i == 300 && ~figure1)
                
                set(h_button1(1), 'BackGroundColor', [0.4,0.4,0.4], 'Enable', 'off');
                set(h_button1(2), 'BackGroundColor', 'b', 'Enable', 'on');
                %drawnow;
                figure1 = true;
            end
            
            if(mod(i,50) == 0), clc; end
            Tekst = sprintf('Trials: %d\nSteps: %d', i, steps1);
            fprintf([Tekst,'\n']);         %写入文本文件
        end

        figure('NumberTitle', 'off', 'Name', '成功率曲线'); 
        plot(1:max_trials/100,arrive_rate);
        
        axis([0 max_trials/100 0 100])
        
        set(gca,'YTick',0:20:100);

        set(gca,'XTick',0:1:10);
        
        xlabel('n/10^{2}');

        ylabel('\xi/%');
    % Main loop for Neural Network method
    elseif( strcmp(mode,'Start2') )
        
        mode = '';
        info_s1 = sprintf('%s%11d%21s%13d%22s%10.2f', 'Trials:', j-1, 'Steps:', steps2, 'Total reward:', reward2);
        
        t1 = clock ;
        
        for j = j:max_trials
            
            figure(1);
            
            subplot(h_plot2(1));
            
            [nn_params,target_params, Q_nn, Q_est, reward2, steps2,percent2, n_crash2, n_arrive2, cost, info_s2, figure_num,learn_pollY,learn_poll_numY] =  ...
                                                                         nnTrials1(nn_params, target_params,input_layer_size, hidden_layer_size,  ...
                                                                         output_layer_size, alpha2, gamma, T2, actions, max_steps, ...
                                                                         n_crash2,n_arrive2, stadium_option2, tot_rot2,h_car2,     ...
                                                                         h_poly2, h_circ2,info_s1, h_text2, init_plot2, goal_list2,...
                                                                         guide_num2, figure_num, learn_pollY,learn_poll_numY);     %3.25改动，加入小车初始位置        
            if (percent2 ==5 )
                
                if  gettable == 0  
                    
                    t2 = clock;
                    first_getTime = etime(t2,t1);
                end
                gettable =gettable+ 1;
                record_table(gettable,:) = [figure_num,(choice2_1+2)*(choice2_2+1),steps2];
            end
                                                                     
            figure(1);  
            
            subplot(h_plot2(1));          %重新画图
            
            delete(h_car2);
            
            choice2_1 = floor(3*rand())+1;   %  随机数乘三向下取整 均匀概率取 1,2,3
            
            guide_list_now2 = guide_list2(:,2*choice2_1-1:2*choice2_1);
            
            init_plot2 = init_plot_list2(choice2_1,:);
            
            choice2_2 = floor(2*rand());
            
            goal_list2= guide_list_now2(guide_num2*choice2_2+1:guide_num2*choice2_2+guide_num2,:);
            
            choice_for_theta2 = floor(theta_num2*rand())+1;   %随机数 生成1 到 theta_num  抽取一个初始角度

            theta2 = theta_list2(choice_for_theta2);
            
            h_car2 = createCarCircle(radius, sensor_lengde, 'b',init_plot2, theta2);
            
            tot_rot2 = theta2;
                %% 动态障碍物运动模块
            if( strcmp(stadium_option2, 'Dynamic') == 1 )
                delete(h_circ2(4));
                delete(h_circ2(5));
                [X,Y] = circle([30, 50], 2, 100);
                cir1_2 = patch(X,Y,'y','EdgeColor', 'b');
                [X,Y] = circle([60, 60], 2, 100);
                cir1_3 = patch(X,Y,'y','EdgeColor', 'b');        %更新两个动态障碍物 
                h_circ2(4)= cir1_2;
                h_circ2(5)= cir1_3;
            end
            

            drawnow;
            
            if( strcmp(mode,'Pause2') ), break; end
            if(T2 > 1.0e-3), T2 = 0.95*T2; end
            
            info_s1 = sprintf('%s%11d%21s%13d%22s%10.2f', 'Trials:', j, 'Steps:', steps2, 'Total reward:', reward2);
            info_s = sprintf('%s\n%s', info_s1, info_s2);
            set(h_text2, 'String', info_s);
            
            n_steps2 = n_steps2 + steps2;
            tot_steps2 = tot_steps2 + steps2;
            total_reward2 = total_reward2 + reward2;
            tot_percent2 = tot_percent2 + percent2;            %累计十步的到达距离半分比
            total_reward2 = total_reward2 + reward2;
            cost_fun(j) = cost;
 %% 输出数据处理
            if (mod(j,100)== 0)
                act = j/100;
                arrive_rate(act) = n_arrive2/5 - sum(arrive_rate); 
            end
            if( mod(j,10) == 0)
                
                n2 = n2 + 1;
                x2(n2) = n2-1;
                
                y3(n2) = tot_percent2*2;
                
                y4(n2) = total_reward2/10;
                
                subplot(h_plot2(2));
                plot(x2,y3);
                xlabel('Trials x10');
                ylabel('Average percentage of distance each 10th trials'); 
                
                subplot(h_plot2(3));
                plot(x2,y4);
                xlabel('Trials x10');
                ylabel('Average reward of each 10th trials');
                total_reward2 = 0;
                tot_percent2 = 0;
                subplot(h_plot2(1));
                drawnow;
            end

%%            
            if(j == 300 && ~figure2)
                
                set(h_button2(1), 'BackGroundColor', [0.4,0.4,0.4], 'Enable', 'off');
                set(h_button2(2), 'BackGroundColor', 'b', 'Enable', 'on');
                figure2 = true;
            end
            
            if(mod(j,50) == 0), clc; end
            Tekst = sprintf('Trials: %d', j);
            fprintf([Tekst,'\n']);            
        end
        figure('NumberTitle', 'off', 'Name', '成功率曲线'); 
        plot(1:max_trials/100,arrive_rate);
        
        axis([0 max_trials/100 0 100])
        
        set(gca,'YTick',0:20:100);

        set(gca,'XTick',0:1:10);
        
        xlabel('n/10^{2}');

        ylabel('\xi/%');

    end 
end

