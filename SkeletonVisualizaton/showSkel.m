function [ ] = show(data, data_pos, figureTitle)
% actual visualization

figure
hold on
lineLen=50;
linewidth=2;
for i=1:11,
    xyz = data(i,[11,12,13]);
    x=xyz(1);
    y=xyz(2);
    z=xyz(3);
    plot3(x,y,z,'r.', 'MarkerSize',15);        
        
    vect = data(i,[1,4,7])*lineLen;
    vect = xyz+vect;
    xd = vect(1);
    yd = vect(2);
    zd = vect(3);
    line([x,xd],[y,yd],[z,zd],'Color','r', 'LineWidth',linewidth);
    
    vect = data(i,[2,5,8])*lineLen;
    vect = xyz+vect;
    xd = vect(1);
    yd = vect(2);
    zd = vect(3);
    line([x,xd],[y,yd],[z,zd],'Color','g', 'LineWidth',linewidth);
    
    vect = data(i,[3,6,9])*lineLen;
    vect = xyz+vect;
    xd = vect(1);
    yd = vect(2);
    zd = vect(3);
    line([x,xd],[y,yd],[z,zd],'Color','b', 'LineWidth',linewidth);    
end

for i=1:4,
    xyz = data_pos(i,[1,2,3]);
    x=xyz(1);
    y=xyz(2);
    z=xyz(3);
    
    plot3(x,y,z,'B.','MarkerSize',15); 
end
axis square
xlabel('x')
ylabel('y')
zlabel('z')
title(figureTitle);
legend('joints','x','y','z');
hold off

fprintf('figure done..\n');

end

