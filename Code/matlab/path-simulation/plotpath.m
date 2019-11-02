function plotpath(x_1,x_2) 
% theta = x_2(3) - x_1(3) ; 
[R,l,sign_R,theta] = line_circle(x_1,x_2) ;
figure
plot(x_1(1),x_1(2),'x','MarkerSize',15) ; 
hold on
plot(x_2(1),x_2(2),'x','MarkerSize',15) ; 
hold on
plot(x_1(1),linspace(0,l,1000),'.b') ;
hold on 
if sign_R > 0  
    th = 0:.001:theta ;
    xunit = R *(1 - cos(th)) + x_1(1);
    yunit = R * sin(th) + l;
else
    th = theta:-.001:0 ;
    xunit = - R *(1 - cos(th)) + x_1(1);
    yunit = R * sin(th) + l;
end
plot(xunit, yunit,'.');
legend('start','end') ;
xlim([-5 5]) 
ylim([-5 5]) 

end
