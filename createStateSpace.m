function state_space = createStateSpace(stadium_option)
	
	%Lage tilstandsrom   ´´½¨×´Ì¬¿Õ¼ä   

    k1 = [0;1;2];
    k2 = [0;1;2];
    k3 = [0;1;2;3];
    k4 = [0;1;2;3];

    kj = [0;1;2;3;4;5];
    if(strcmp(stadium_option, 'Static')), state_space = setprod(k1, k2, k3, k4 , kj); 
    
    elseif(strcmp(stadium_option, 'Dynamic')), state_space = setprod(k1, k2, k3, k4, kj);     
    
    end 
    
end