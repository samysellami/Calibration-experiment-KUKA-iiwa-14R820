function [q] = RandomConfig_Kuka(qNum, lowLimit, upLimit)

for i = 1:qNum
    q(i,:) = [rand(1)*(-lowLimit(1)+upLimit(1)) + lowLimit(1),...
        rand(1)*(-lowLimit(2)+upLimit(2)) + lowLimit(2),...
        rand(1)*(-lowLimit(3)+upLimit(3)) + lowLimit(3),...
        rand(1)*(-lowLimit(4)+upLimit(4)) + lowLimit(4),...
        rand(1)*(-lowLimit(5)+upLimit(5)) + lowLimit(5),...
        rand(1)*(-lowLimit(6)+upLimit(6)) + lowLimit(6),...
        rand(1)*(-lowLimit(7)+upLimit(7)) + lowLimit(7)];
end