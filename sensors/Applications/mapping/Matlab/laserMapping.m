% Laser mapping application
% Angelos Plastropoulos
%
% RANSAC algorithm is implemented by Yan Ke @ THUEE, xjed09@gmail.com
% distrubuted for free use as ransac_homography library
% http://uk.mathworks.com/matlabcentral/fileexchange/30809-ransac-algorithm-with-example-of-finding-homography

clear all;
close all;
clc;

% set the max and min laser range
% and sweep angle
MINDIST = 0;
MAXDIST = 4000;
MINANGLE = 0;
MAXANGLE = 180;

count = 1; % incremental index
ptNum = 180; % number of scan points
points = zeros(2,ptNum); % array of points in (x,y)
X = -ptNum:ptNum;

% declare the serial communication port
s = serial('/dev/cu.usbmodem1421');
set(s,'Baudrate', 9600);
set(s,'DataBits', 8);
set(s,'StopBits', 1);
set(s,'Parity', 'none');

% open the port in async
fopen(s);
s.ReadAsyncMode = 'continuous';    

readasync(s);

fprintf('Measurements\n');

% make a sweep back and forth, since it measures
% for 90 points from 0 to 180 degrees
while(count < ptNum)
    angle = fscanf(s, '%d');
    dist = fscanf(s, '%d');
    
    distCheck = size(dist);
    angleCheck = size(angle);
    
    % safaty checks for distance and angle messages
    if( (distCheck(1, 1) == 1)  & (distCheck(1,2) == 1) & (angleCheck(1,1) == 1)...
            & (angleCheck(1,2) == 1) & (dist >= MINDIST) & (dist <= MAXDIST)...
            & (angle >= MINANGLE) & (angle <= MAXANGLE) )
        
        %convert from polar to cartesian system
        x = dist*cos((pi/180.0)*angle);
        y = dist*sin((pi/180.0)*angle);
        
        % populate the array thats stores the laser scan (x,y) points
        points(1, count) = x;
        points(2, count) = y;
        axis equal
        hold on
        % plot the each point
        plot(x,y,'o')
        drawnow
    end
       
   %fprintf('angle = %d dist = %d\n', angle, dist);
    count = count +1;
end

iterNum = 300; % the number of iteration
thDist = 2; % the inlier distance threshold
thInlrRatio = .1; % the inlier number threshold

firstTime = 1; % useful flag

% execute the RANSAC for every group of 10 points
for kk = 1:10:ptNum
    [t,r] = ransac(points(:,kk:(kk+9)),iterNum,thDist,thInlrRatio);
    
    if (firstTime == 0)
        thetaOld = thetaNew;
        thetaNew = t;
    else
        thetaNew = t;
    end
    
    % draw only lines that differ 5 degrees in direction
    if (firstTime == 0)
        dif = thetaNew - thetaOld;
        if (abs(dif) < 5*(pi/180))
            continue;
        end
    end
    
    k1 = -tan(t);
    b1 = r/cos(t);
    % plot
    plot(X,k1*X+b1,'r')
    
    firstTime = 0;
end

% close the port
stopasync(s);
fclose(s);
fprintf('Communication closed\n');


    