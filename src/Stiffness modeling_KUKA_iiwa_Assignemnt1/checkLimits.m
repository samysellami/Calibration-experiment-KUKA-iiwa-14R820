function [r] = checkLimits(q)
%%%%%%  Homework 1%%%%%%
%%%%%% Victor Massague , Sami SELLAMI %%%%%%%
%%%%%% function that checks the limits of the KUKA iiwa manipulator angles %%%%%%%%%%
%%%%%% input: angles q %%%%%%%%%%%%%%%%%
%%%%%% output: True if configuration is inside limits   %%%%%%%%%%%%%%%%%%%%%%%%%%%%

r = 0;
if isreal(q(1)) && isreal(q(2)) && isreal(q(3)) && isreal(q(4)) && isreal(q(5)) && isreal(q(6)) && isreal(q(7))...
    &&abs(q(1)) <= deg2rad(170) && abs(q(2)) <= deg2rad(120) && abs(q(3)) <= deg2rad(170) ...
    && abs(q(4)) <= deg2rad(120) && abs(q(5)) <= deg2rad(170) && abs(q(6)) <= deg2rad(120)...
    && abs(q(7)) <= deg2rad(175)
    r = 1;
end
end

