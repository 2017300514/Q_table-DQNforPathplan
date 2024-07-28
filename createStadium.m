function [h_polygons, h_circles,init_plot_list,guide_list_all,guide_num,theta_list,theta_num] = createStadium(mode)
    
    %% ----- Initialize the obstacles and the arena  ��ʼ���ϰ��Ϳռ�վ��ͼ

    h_polygons = [];     %������ϰ�
    h_circles = [];      %Բ���ϰ�
    circle_r1 = 5;  
    circle_r2 = 2;
    %%  ---��̬�ϰ����ͼ    
    if( strcmp(mode,'Static')  )
        
        %% ��Ҫ��ͼ
        %�����κ�ɫ���α���
        X1=[0,0,100,100];
        Y1=[0,100,100,0];
        
        area1=patch(X1,Y1,'k');
        % ƽ��ͼ(��������ն�1,���ܲն�2,3,4,�����Ӵ�5,6,7)
        X2=[0,  0, 12.5,12.5,7.5,7.5,27.5,27.5,22.5,22.5,90,90,85,85,90,90,70,70,75,75,22.5,22.5,27.5,27.5,7.5,7.5,12.5,12.5];
        Y2=[40, 65,65,  70,  70, 100,100, 70,  70,  65,  65,40,40,35,35,10,10,35,35,40,40,  35,  35,  0,   0,  35, 35,  40];
        
        area2=patch(X2,Y2,'w');
        
        %% �������ϰ�
        %      ������� �ϰ�1 2 3
        X1=[0,0,5,5];
        Y1=[40,65,65,40];    %������
        square1_1=patch(X1,Y1,'c','EdgeColor','b');     
    
        %       ����2   �ϰ�1
        X1=[7.5,16,16,7.5];
        Y1=[82.5,82.5,87.5,87.5];   
        square2_1=patch(X1,Y1,'g','EdgeColor','b');
        %       ����3   �ϰ�1
        X1=[20,20,27.5,27.5];
        Y1=[0,10,10,0];    
        square3_1=patch(X1,Y1,'g','EdgeColor','b');
       
        
        %% �������ϰ� 
        %       ����2   �ϰ�1
        X1 = [20, 100; 27.5, 100; 27.5, 85];
        triangle2_1 = patch(X1(:,1), X1(:,2), 'g');
        %       ����4   �ϰ�1
        X1 = [76, 10; 84, 10; 80, 22.5];
        triangle4_1 = patch(X1(:,1), X1(:,2), 'g');  
        
        
        %% Բ���ϰ�
        %       ������� �ϰ�1
        [X,Y] = circle1([90, 65], 2*circle_r1,[pi,1.5*pi], 400);
        cir1_1 = patch(X,Y,'r','EdgeColor', 'b');  
        %       ����3   �ϰ�1
        [X,Y] = circle1([7.5, 17.5], circle_r1,[-0.5*pi,0.5*pi], 200);
        cir3_1 = patch(X,Y,'r','EdgeColor', 'b');
        %       ����4   �ϰ�1
        [X,Y] = circle1([90, 35], circle_r1,[pi,1.5*pi], 200);
        cir4_1 = patch(X,Y,'r','EdgeColor', 'b');

   
        %% ������
        h_polygons = [triangle2_1,triangle4_1,square1_1,square2_1,square3_1,area2,area1];  

        h_circles = [cir1_1,cir3_1,cir4_1];                   
    %% ��̬�ϰ����ͼ
    elseif( strcmp(mode,'Dynamic') )
        %% ��Ҫ��ͼ
        %�����κ�ɫ���α���
        X1=[0,0,100,100];
        Y1=[0,100,100,0];
        
        area1=patch(X1,Y1,'k');
        % ƽ��ͼ(��������ն�1,���ܲն�2,3,4,�����Ӵ�5,6,7)
        X2=[0,  0, 12.5,12.5,7.5,7.5,27.5,27.5,22.5,22.5,90,90,85,85,90,90,70,70,75,75,22.5,22.5,27.5,27.5,7.5,7.5,12.5,12.5];
        Y2=[40, 65,65,  70,  70, 100,100, 70,  70,  65,  65,40,40,35,35,10,10,35,35,40,40,  35,  35,  0,   0,  35, 35,  40];
        
        area2=patch(X2,Y2,'w');
        
        %% �������ϰ�
        %      ������� �ϰ�1 2 3
        X1=[0,0,5,5];
        Y1=[40,65,65,40];    %������
        square1_1=patch(X1,Y1,'c','EdgeColor','b');     
    
        %       ����2   �ϰ�1
        X1=[7.5,16,16,7.5];
        Y1=[82.5,82.5,87.5,87.5];   
        square2_1=patch(X1,Y1,'g','EdgeColor','b');
        %       ����3   �ϰ�1
        X1=[20,20,27.5,27.5];
        Y1=[0,10,10,0];    
        square3_1=patch(X1,Y1,'g','EdgeColor','b');
       
        
        %% �������ϰ� 
        %       ����2   �ϰ�1
        X1 = [20, 100; 27.5, 100; 27.5, 85];
        triangle2_1 = patch(X1(:,1), X1(:,2), 'g');
        %       ����4   �ϰ�1
        X1 = [76, 10; 84, 10; 80, 22.5];
        triangle4_1 = patch(X1(:,1), X1(:,2), 'g');  
        
        
        %% Բ���ϰ�
        %       ������� �ϰ�1
        [X,Y] = circle1([90, 65], 2*circle_r1,[pi,1.5*pi], 400);
        cir1_1 = patch(X,Y,'r','EdgeColor', 'b');  
        %       ����3   �ϰ�1
        [X,Y] = circle1([7.5, 17.5], circle_r1,[-0.5*pi,0.5*pi], 200);
        cir3_1 = patch(X,Y,'r','EdgeColor', 'b');
        %       ����4   �ϰ�1
        [X,Y] = circle1([90, 35], circle_r1,[pi,1.5*pi], 200);
        cir4_1 = patch(X,Y,'r','EdgeColor', 'b');


        %% ��̬Բ���ϰ�
        %       ������� �ϰ�2
        [X,Y] = circle([30, 50], 2*circle_r2, 100);
        cir1_2 = patch(X,Y,'y','EdgeColor', 'b');
        %       ������� �ϰ�3
        [X,Y] = circle([60, 60], 2*circle_r2, 100);
        cir1_3 = patch(X,Y,'y','EdgeColor', 'b');
        
        %% ������
        h_polygons = [triangle2_1,triangle4_1,square1_1,square2_1,square3_1,area2,area1];  

        h_circles = [cir1_1,cir3_1,cir4_1,cir1_2,cir1_3]; 
        
        % ������ʼ���б�

    end 
    init_plot_list = [13,7;14,93;76,22.5];
%     init_plot_list = [80,30;87,16;73,16];
    guide_num = 5;
    theta_num = 12;
    theta_list = [0,pi/6,pi/3,pi/2,2*pi/3,5*pi/6,pi,-pi/6,-pi/3,pi/2,2*pi/3,5*pi/6];
    guide_list1 = [20,17.5;13,30;17.5,55;80,   55;73,16;20,17.5;13,30;17.5,55;14,76;14,93];
    guide_list2 = [20,  85;14,76;17.5,55;13,   30;13, 7;20,  85;14,76;17.5,55;80,  55;87,16];
    guide_list3 = [87,22.5;80,55;17.5,55;14,   55;14,93;80,  25;87,22.5;80  ,55;17.5,55;13,7];
    guide_list_all  = [guide_list1,guide_list2,guide_list3];
end