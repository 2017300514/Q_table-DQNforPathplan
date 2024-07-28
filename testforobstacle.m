

% [X,Y] = circle([10, 46], 1, 100);
% cir4_1 = patch(X,Y,'r','EdgeColor', 'b');
% car = get(cir4_1, 'Vertices');
% [X,Y] = circle1([90, 40], 5,[pi,1.5*pi], 200);
% cir3_1 = patch(X,Y,'r','EdgeColor', 'b');
% obstacle = get(cir3_1, 'Vertices');
%dist_crash  ��������ײ��ⷶΧ
obstacle=[0,0,12.5,12.5,10,10,25,25,22.5,22.5,90,90,85,85,90,90,70,70,75,75,22.5,22.5,25,25,10,10,12.5,12.5;45,65,65,75,75,100,100,75,75,65,65,45,45,40,40,15,15,40,40,45,45,40,40,0,0,40,40,45];
obstacle = obstacle';
% obstacle =[10, 85; 10, 90; 16, 87.5];
% function crash = obstacleCrash1(car, obstacle, shape)

%�����ײ�ϰ���ĺ���������һ������ֵ
%car С��ģ�͵Ķ�������
%obstacle  �ϰ��ﶥ������
% shape    �ϰ�����״

crash = false;          %    ��ʼ����ײ�ж�Ϊ��
shape = 'polygon';
P = obstacle;           %    �ϰ��ﶥ������
x = car(:,1);           %    С������ x ����
y = car(:,2);           %    С������ y ����
dist_between = 0.5;     %    �涨����ײ����
n_punkter = size(car,1);
x0 = car(n_punkter,1);
y0 = car(n_punkter,2);
dist_value=10;

centre = [x0, y0];    %����С��Բ��λ��
radius = norm([car(1,1) - x0, car(1,2) - y0] );  %����С���뾶
min_dist=radius+dist_between;
%��������� �涨������ϰ뾶����Ϊ���������붼�Ǵ���Բ�ĵ�

% �ж��ϰ�����״
if( strcmp(shape,'circle') )
    n_punkter = size(P,1);
    a1 = P(n_punkter,1);
    b1 = P(n_punkter,2);

    
    radius_obs = norm([P(1,1) - a1, P(1,2) - b1] );  %����Բ���ϰ���뾶
    
    centre_dist = (x0-a1)^2+(y0-b1)^2;   %����Բ�ľ�
    
    
    if( centre_dist<(radius_obs+radius+dist_between)^2)
        %���Բ�ľ�С�ڰ뾶֮�ͼ���ײ���룬���ж�Ϊ��ײ
        crash = true;
    end
    
    %  ���ڶ�����ϰ���  ����������ϰ�����ϣ����ش�ֱ���룬���򷵻� ���������˵���������
elseif( strcmp(shape,'polygon') )
    
    n_punkter = size(P,1);     %����ζ�������

    for i = 1:n_punkter-1   % ����������n-1����
        b = P(i+1, 2) - P(i, 2);
        c = P(i+1, 1) - P(i, 1);
        if( abs(c) < 0.001 );kkp_value = false; else; kkp_value = true;  kk_p  = b/c; b_p = -kk_p * P(i, 1) + P(i, 2);         end
        
        if( abs(b) < 0.001 );kkc_value = false; else; kkc_value = true;  kk_c=-c/b;  b_c = -kk_c * x0 + y0;           end
        
        x_limit1 = min([P(i,1), P(i+1,1)]);     % �߳�����˵�x����
        x_limit2 = max([P(i,1), P(i+1,1)]);     % �߳����Ҷ˵�x����
        y_limit1 = min([P(i,2), P(i+1,2)]);     % �߳�������˵�y����
        y_limit2 = max([P(i,2), P(i+1,2)]);     % �߳�������˵�y����   �⼸�������������жϴ����Ƿ��ڱ߳���
        vertice1=[P(i,1),P(i,2)];
        vertice2=[P(i+1,1),P(i+1,2)];          %��¼�����˵�����
        if(~kkc_value)
            dist_patch = (y0 - P(i,2))^2;
        elseif(~kkp_value)
            dist_patch = (x0-P(i, 1))^2;
        else                 %�������α߳��Ǵ�ֱ�ģ�����x����Ĳ��ƽ��
            dist_patch = dist_pointToline(kk_p,b_p,centre);     %���㴹ֱ�����ƽ�����õ㵽ֱ�߾��빫ʽ
        end
        %         if(kkp_value)
        %             dist_patch = dist_pointToline(kk_p,b_p,centre);     %���㴹ֱ�����ƽ�����õ㵽ֱ�߾��빫ʽ
        %         else
        %             dist_patch = (x0-P(i, 1))^2;                 %�������α߳��Ǵ�ֱ�ģ�����x����Ĳ��ƽ��
        %         end
        if(~kkc_value)
            x_vertical = x0;    %���������б�ʣ�����Բ�ĺ�����
            if(x_vertical>=x_limit1 && x_vertical<=x_limit2)    %��������ڱ߳���Χ�ڣ����ش��߶γ��ȣ����򷵻ص����������ľ���
                dist_temp = sqrt(dist_patch);
            else
                dist_temp = min([pdist([vertice1;centre],'euclidean'),pdist([vertice2;centre],'euclidean')]);
            end
            
        elseif(~kkp_value)
            y_vertical = y0;    %  �������α߳���ֱx�ᣬ ������㴹��������
            if(y_vertical>=y_limit1 && y_vertical<=y_limit2)
                dist_temp = sqrt(dist_patch);
            else
                dist_temp = min([pdist([vertice1;centre],'euclidean'),pdist([vertice2;centre],'euclidean')]);
            end
        else
            x_vertical = (b_c-b_p)/(kk_p-kk_c);     %���ش���ĺ�����
            if(x_vertical>=x_limit1 && x_vertical<=x_limit2)   %������������ڷ�Χ�ڣ����ش�ֱ����
                dist_temp = sqrt(dist_patch);
            else
                dist_temp = min([pdist([vertice1;centre],'euclidean'),pdist([vertice2;centre],'euclidean')]);
                %����������겻�ڷ�Χ�ڣ���������Ķ˵����
            end
        end
        
        
        if(dist_value>dist_temp); dist_value=dist_temp;end    % ������̵ľ���ֵ��
    end
    
    
    
    %���ڶ�����ϰ������һ����
    
    b = P(n_punkter, 2) - P(1, 2);
    c = P(n_punkter, 1) - P(1, 1);
    if( abs(c) < 0.001 );kkp_value = false; else; kkp_value = true;  kk_p  = b/c;  b_p = -kk_p * P(1, 1) + P(1, 2);       end
    if( abs(b) < 0.001 );kkc_value = false; else; kkc_value = true;  kk_c=  -c/b;  b_c = -kk_c * x0 + y0;           end
    x_limit1 = min([P(1,1), P(n_punkter,1)]);     % �߳�����˵�x����
    x_limit2 = max([P(1,1), P(n_punkter,1)]);     % �߳����Ҷ˵�x����
    y_limit1 = min([P(1,2), P(n_punkter,2)]);     % �߳�������˵�y����
    y_limit2 = max([P(1,2), P(n_punkter,2)]);     % �߳�������˵�y����   �⼸�������������жϴ����Ƿ��ڱ߳���
    vertice1=[P(1,1),P(1,2)];                     %�˵�1 ����
    vertice2=[P(n_punkter,1),P(n_punkter,2)];     %�˵�2 ����
    
    
    if(~kkc_value)
        dist_patch = (y0 - P(1,2))^2;
    elseif(~kkp_value)
        dist_patch = (x0-P(1, 1))^2;
    else                 %�������α߳��Ǵ�ֱ�ģ�����x����Ĳ��ƽ��
        dist_patch = dist_pointToline(kk_p,b_p,centre);     %���㴹ֱ�����ƽ�����õ㵽ֱ�߾��빫ʽ
    end
    if(~kkc_value)
        x_vertical = x0;     %���������б�ʣ�����Բ�ĺ�����
        
        if(x_vertical>=x_limit1 && x_vertical<=x_limit2)    %��������ڱ߳���Χ�ڣ����ش��߶γ��ȣ����򷵻ص����������ľ���
            dist_temp = sqrt(dist_patch);
        else
            dist_temp = min([pdist([vertice1;centre],'euclidean'),pdist([vertice2;centre],'euclidean')]);
        end
        
    elseif(~kkp_value)
        y_vertical = y0;    %  �������α߳���ֱx�ᣬ ������㴹��������
        if(y_vertical>=y_limit1 && y_vertical<=y_limit2)
            dist_temp = sqrt(dist_patch);
        else
            dist_temp = min([pdist([vertice1;centre],'euclidean'),pdist([vertice2;centre],'euclidean')]);
        end
    else
        x_vertical = (b_c-b_p)/(kk_p-kk_c);
        if(x_vertical>=x_limit1 && x_vertical<=x_limit2)
            dist_temp = sqrt(dist_patch);
        else
            dist_temp = min([pdist([vertice1;centre],'euclidean'),pdist([vertice2;centre],'euclidean')]);
        end
    end
    
    
    if(dist_value>dist_temp); dist_value=dist_temp;end    % ������̵ľ���ֵ
    
    
    if(dist_value< min_dist)
        crash = true;
    end
    
end


