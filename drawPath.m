function drawPath(z, path_x, path_y, mode)
figure(z);

[h_poly2, h_circ2] = createStadium(mode);
hold all
plot(path_x, path_y, 'r--o');
% path_x = [1, 2, 3, 4];
% path_y = [4, 3, 2, 1];
end
   

