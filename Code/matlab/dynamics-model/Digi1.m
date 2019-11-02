sys = tf(100,[1 0 0]);
%a = 4
sysc=tf([1 4],1);
syso = sysc*sys;
rlocus(syso)
