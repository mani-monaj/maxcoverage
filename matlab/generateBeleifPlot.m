function generateBeleifPlot(filename, plottitle)
data = csvread(filename);
[row, col] = size(data);
t = data(:,1);
n = data(:,4);
rawb = data(:,5:col-1);
b = zeros(row,1);
s = zeros(row,1);

err = 0.0;
errc = 0;
for i = 1:row
    oneb = rawb(i, find(rawb(i,:) > 0)); 
    if (n(i) == 0)
        b(i,1) = 0;
        s(i,1) = 0;
    else
        for j = 1:length(oneb)
            err = err + ((oneb(j) - n(i)) ^ 2);
            errc = errc + 1;
        end
        b(i,1) = mean(oneb);
        s(i,1) = std(oneb);
    end
    
end
%b = mean(rawb,2);
%s = std(rawb,0,2);

%err = (b - n).^2;
%err = sqrt(sum(err) / row);
err = sqrt(err / errc);
fprintf('RMS Error: %6.4f\n', err);
ub = b + s;
lb = b - s;

plot(t, ub, 'color', [0.8 0.8 0.8], 'Marker', '.', 'LineStyle', 'none', 'MarkerSize', 6);
plot(t, lb, 'color', [0.8 0.8 0.8], 'Marker', '.', 'LineStyle', 'none', 'MarkerSize', 6);
hold on;
plot(t,n,t,b);
hold off;
title(plottitle);
ylabel('Number of Robots');
xlabel('time (ms)');
legend('Std Dev','Actual Number of robots','Mean of robot beleifs');
grid on;


end