x = linspace(0,255,256);
gamma = [0.04 0.1 0.25 0.45 0.7 1 1.5 2.5 5 10 25];
hold on 
for i = 1:11
    gm = gamma(i);
    y = (x/255).^gm .*255;
    plot(x, y);
    xlim([0,255]);
    ylim([0,255]);
end
xlabel("Input Pixel Value");
ylabel("Output Pixel Value");