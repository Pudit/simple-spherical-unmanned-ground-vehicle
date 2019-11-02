function [R,l,sign_R,theta] = line_circle(x_1,x_2) 
dx = x_2(1) - x_1(1) ;
dy = x_2(2) - x_1(2) ; 
theta = x_2(3) - x_1(3) ; 
R  = dx/(1-cos(theta)) ;

if (0 <= R) && (R < 0.54) 
    R = 0.54 ;
    theta = acos(1 - (dx)/R) ;
else if (0 >R ) && ( R > -.54)
    R = -0.54 ;
    theta = acos(1 - (dx)/R) ;
   
end
sign_R = sign(R) ;
R = abs(R) ; 

l = dy - R*sin(theta) ; 

    
end 