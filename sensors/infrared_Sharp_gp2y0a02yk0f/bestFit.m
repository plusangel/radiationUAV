% Best-fit equation from Sharp's graph Voltage (v) to Distance (d)
% 

d = [20  30  40  50  60  70  80  90  100 110 120 130 140 150];
v = [512 409 306 255 214 184 163 153 133 122 112 102 98  92];

p = fit(d',v','power2')

plot(p, d', v','o');
