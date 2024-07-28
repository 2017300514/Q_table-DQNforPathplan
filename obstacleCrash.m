function crash = obstacleCrash(car, obstacle, shape)
    
    %�����ײ�ϰ���ĺ���������һ������ֵ
    %car С��ģ�͵Ķ�������
    %obstacle  �ϰ��ﶥ������
    % shape    �ϰ�����״
  	crash = false;
    
	P = obstacle;           %    �ϰ��ﶥ������
	x = car(:,1);           %   С������ x ����
	y = car(:,2);           %   С������ y ����
    dist_between = 0.3;     
    
    % �ж��ϰ�����״
    if( strcmp(shape,'circle') )
                
		a = (min(P(:,1)) + max(P(:,1)))/2;
        b = (min(P(:,2)) + max(P(:,2)))/2; 
        
        centre = [a, b];    %�����Բ���ϰ��ͨ��ȡƽ������Բ��λ��
        radius = norm([P(1,1) - centre(1), P(1,2) - centre(2)] );  %����뾶
        
        %��������С������������ӦԲ�ϵ�y���꣬��ΪԲ�Գ��Եõ�����
		f1 =  sqrt( radius.^2 - (x - a).^2 ) + b;
		f2 = -sqrt( radius.^2 - (x - a).^2 ) + b;  %sqrt���븺���õ�����
        
        % �õ��������ԳƵ���������
		g1 = abs(y-f1);
		g2 = abs(y-f2);   %���������õ�ģ��  
        
        %Բ
        if( ~isempty(find( g1 < dist_between, 1)) || ~isempty(find( g2 < dist_between, 1)) )
            
            crash = true;
        end

    %  ���ڶ�����ϰ���
    elseif( strcmp(shape,'polygon') )

		n_punkter = size(P,1);
        
        % f(x) = a*(x-x0) + f(x0)   �������� ��n-1����
        for i = 1:n_punkter-1  
            
            g = [];
            b = P(i+1, 2) - P(i, 2);
            c = P(i+1, 1) - P(i, 1);
            
            if( c ~= 0 )
            
                a = b/c;

                x_limit1 = min([P(i,1), P(i+1,1)]);         %�߶���˵�
                x_limit2 = max([P(i,1), P(i+1,1)]);         %�߶��Ҷ˵�
                
                
                I = find((x_limit1 < x) & (x < x_limit2) );     %�����������������˵�֮��ĺ�����֮�䣬����¼�ж�ֵ
                
                if( ~isempty(I) )
                    
                    f = a.*(x(I) - P(i,1)) + P(i,2);        %function f(x)
                    g = abs(y(I) - f);                      % ����y����ֵ
                end

            else    %���б������󣬼�����α߳���ֱx�� ����x����Ϊ����
                
                y_limit1 = min([P(i,2), P(i+1,2)]);         %y-min grense
                y_limit2 = max([P(i,2), P(i+1,2)]);         %y-maks grense
                
                % �����������������˵�֮�����ֱ����֮�䣬����¼�ж�ֵ
                I = find( (y_limit1 < y) & (y < y_limit2) );    
                
                if( ~isempty(I) )
                    
                    
                    g = abs( x(I) - P(i,1) );
                end
                
            end
			
            % �жϾ����Ƿ�ﵽ��ײ
            if( ~isempty(find( g < dist_between, 1)) )
                    
                crash = true;
                return;
            end
            
        end
        % �������ε����һ���߳�
        g = [];
        b = P(n_punkter, 2) - P(1, 2);
        c = P(n_punkter, 1) - P(1, 1);
        
        if(c ~= 0)
            
            a = b/c;
            
            x_limit1 = min([P(1,1), P(n_punkter,1)]);
            x_limit2 = max([P(1,1), P(n_punkter,1)]);
            
            I = find((x_limit1 < x) & (x < x_limit2) );
            
            if( ~isempty(I) )
                
                f = a*( x(I) - P(1,1)) + P(1,2);
                
                g = abs(y(I) - f);
            end
                    
        else      
            
            y_limit1 = min([P(1,2), P(n_punkter,2)]);
            y_limit2 = max([P(1,2), P(n_punkter,2)]);
            
            I = find( (y_limit1 < y) & (y < y_limit2) );
            
            if( ~isempty(I) )
                
                g = abs( x(I) - P(1,1) );
            end
            
        end
        
        if( ~isempty(find( g < dist_between, 1)) )

			crash = true;
        end
    end
end
