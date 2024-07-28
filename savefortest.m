figure(1)
hold on
x1 = [5,15,25,35,45,55];
            
y1 = [43,80.8,66.4,93,93,95.8];

y2 = [360.039966000000,970.613647999999,830.951906999999,1113.01468400000,1122.06405000000,1189.21969100000];

subplot(h_plot1(2));
plot(x1,y1);
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