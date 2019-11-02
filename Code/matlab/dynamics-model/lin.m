%linearization 
%let x_0 = [alpha_0 ; psi_0]' 
%f(x) ~ f(x_0) + df(x_0)' * x - df(x_0)' * x_0
syms alpha_p_dot_0  psi_dot_0 alpha_p_0  psi_0
x_int = [ psi_dot_0 ;alpha_p_dot_0  ; psi_0 ;alpha_p_dot_0 ] ;
B_T_0 = subs(B_T,q_state,x_int) ; 
for eq = 1:size(L,1)
    for i = 1:size(q_state,1)
        d_tmp = diff(state(eq),q_state(i)) ;
        df_x_0 = subs(d_tmp,q_state,x_int) ; 
        f_x_0 = subs(state(eq),q_state,x_int) ; 
        if i == 1 
            B(eq,1) = f_x_0 - df_x_0;
        else 
            B(eq,1) = B(eq,1) - df_x_0; 
        end
       
        A(eq,i) = df_x_0 ;
        
    end
end
   
for eq = size(A,1)+1:1:size(A,2) 
    for i = 1:size(A,2)
        if eq == i
            A(eq,i) = 1 ;
        else 
            A(eq,i) = 0 ;
        end
        
    end
    B(eq,1) = 0 ;
end
B = B + [B_T_0*T ; 0 ; 0] ;