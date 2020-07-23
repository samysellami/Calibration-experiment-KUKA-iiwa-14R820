function [] = showDeflection(F, method)
warning('off','all')
x = -500:50:500;
y = -500:50:500;
z = 1.0;
d = zeros(length(x),length(y));

if strcmp(method,'MSA')
    for i = 1:length(x)
        for j = 1:length(y)
            if abs(x(i)) >= 50 || abs(y(j)) >= 50
                % Calc robot stiffnes matrix
                Kc = get_kcMSA(x(i)/1000, y(j)/1000, z);       
                dr = Kc*F;
                dr = sqrt(dr(1)^2+dr(2)^2+dr(3)^2);
                d(i,j) = dr;
            else
               d(i,j) = 0; 
            end
        end    
    end
else
    for i = 1:length(x)
        for j = 1:length(y)
            if abs(x(i)) >= 50 || abs(y(j)) >= 50
                % Calc robot stiffnes matrix
                Kc = get_kcVJM(x(i)/1000, y(j)/1000, z);
                dr = Kc*F;
                dr = sqrt(dr(1)^2+dr(2)^2+dr(3)^2);
                d(i,j) = dr;
            else
               d(i,j) = 0; 
            end
        end    
    end
end
figure()
contourf(d',[0.000001 0.000001 0.0001 0.001 0.01 0.1])

end