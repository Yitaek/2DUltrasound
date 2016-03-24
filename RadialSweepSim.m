clear; 

angle = linspace(-135, -45, 90);
r = linspace(0, 1, 10);

figure(1); clf
hold on
for i=1:length(angle)
   
    for n=1:length(r)
        x = (255/5).*(r(n).*cosd(angle(i)) + 1);
        y = (255/5).*(r(n).*sind(angle(i)) + 1);
        scatter(x,y)
        
    end
end


% figure(1); clf
% plot(x,y)