clc;
X = zeros(2,1000);

X(:,1) = [0.1 ; 0];

for i=1:1000
    X(:,i+1) = RungeKuttaFixedTime(@SpringMassState,X(:,i),i/100,0.01);
end

plot (X(1,:));