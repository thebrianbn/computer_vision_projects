function project2_newest(jointsinput, vueinput1, vueinput2)
profile on;
% Load in provided data
if ~exist('jointsinput', 'var')
    jointsinput = 'Subject4-Session3-Take4_mocapJoints.mat';
end
if ~exist('vueinput1', 'var')
    vueinput1 = "vue2CalibInfo.mat";
end
if ~exist('vueinput2', 'var')
    vueinput2 = "vue4CalibInfo.mat";
end
m = open(jointsinput);
v2 = open(vueinput1);
v4 = open(vueinput2);

% Subset joints data
joints = m.mocapJoints;

ids = [];
for frame = 1:size(joints,1)
    if not(any(joints(frame,1:12,4) == 0))
       ids = [ids frame];
    end   
end
joints_new = joints(ids,:,1:3);

% Parameters and matrices for vue2
prinpoints = v2.vue2.prinpoint;
xprin = prinpoints(1);
yprint = prinpoints(2);
rotation = v2.vue2.Rmat;
position = v2.vue2.position;
focal_length =  v2.vue2.foclen;
pixelmat = [1 0 xprin; 0 -1 yprint; 0 0 1];
Pmat = v2.vue2.Pmat;
Pmat = [Pmat;0 0 0 1];

% Adjust rotation matrix
rotation_matrix = [rotation; 0,0,0];
temp = [0;0;0;1];
rotation_matrix = [rotation_matrix temp];

% Create skew matrix
skewmatrix = [1 0 0; 0 1 0; 0 0 1; 0 0 0];
position = position*-1;
position = [position 1]';
skewmatrix = [skewmatrix position];

% Parameters and matrices for vue4
prinpoints4 = v4.vue4.prinpoint;
xprin4 = prinpoints4(1);
yprin4 = prinpoints4(2);
rotation4 = v4.vue4.Rmat;
position4 = v4.vue4.position;
focal_length4 =  v4.vue4.foclen;
pixelmat4 = [1 0 xprin4; 0 -1 yprin4; 0 0 1];
Pmat4 = v4.vue4.Pmat;
Pmat4 = [Pmat4;0 0 0 1];




% Forward projection for each vue for all 12 joints
A = zeros(3,size(joints_new,1),12);
B = zeros(3,size(joints_new,1),12);
for y = 1:12
    for x = 1:size(joints_new,1)
        point = joints_new(x,y,1:3);
        newpoint = world2camera(Pmat, [point(1); point(2);point(3);1]);
        pixelpoint = camera2pixel([v2.vue2.Kmat [0;0;0]], newpoint);
        A(:,x,y) = pixelpoint;
        
        newpoint4 = world2camera(Pmat4, [point(1);point(2);point(3);1]);
        pixelpoint4 = camera2pixel([v4.vue4.Kmat [0;0;0]],newpoint4);
        B(:,x,y) = pixelpoint4;
    end
end


% Load in mp4 movie file
filenamevue4mp4 = 'Subject4-Session3-24form-Full-Take4-Vue4.mp4';


% Calculate camera ray for vue2
Kmat = v2.vue2.Kmat;
kInv = inv(Kmat);

% Calculate camera ray for vue4
Kmat4 = v4.vue4.Kmat;
kInv4 = inv(Kmat4);

C = zeros(3,size(joints_new,1),12);
for y = 1:12
    for x = 1:size(joints_new,1)
        cameraRay = rotation' * kInv * A(1:3,x,y);
        cameraRay4 = rotation4' * kInv4 * B(1:3,x,y);
        % Calculate u3 and point
        u3 = u3getter(cameraRay, cameraRay4);
        [a,b,d] = triangulator(cameraRay,cameraRay4,u3,position(1:3),position4(1:3));
        answer = pcalculator(cameraRay, cameraRay4,a,b,position(1:3),position4(1:3));
        answer = -answer;
        C(:,x,y) = answer;
 
    end
end

distances = zeros(12,size(joints_new,1));
for a = 1:12
    for z = 1:size(joints_new,1)
        distances(a,z) = euclidean_distance(joints_new(z,a,1:3), C(1:3,z,a));
        
        
    end
        
end
    

joint = 5;
for frame = [16451, 23442, 765, 2424, 100, 250, 400, 550, 700]
    disp(["This data is from frame", ids(frame)])
    displayframewithSkeleton(frame, A, ids)
    
    
    
    point = joints_new(frame,joint,1:3);
    point = point(:);
    point = [point;1];
    cp = world2camera(Pmat,point);
    cp4 = world2camera(Pmat4, point);
    disp('This is the camera value for joint 5 vue2:');
    disp(cp);
    disp('This is the camera value for joint 5 vue4:');
    disp(cp4);
    pp = camera2pixel([Kmat [0;0;0]], cp);
    pp4 = camera2pixel([Kmat4 [0;0;0]],cp4);
    disp('This is the pixel value for joint 5 vue2:');
    disp(pp);
    disp('This is the pixel value for joint 5 for vue4:');
    disp(pp4);
    
    
    cameraRay = rotation' * kInv * pp;
    disp("The ray for vue2")
    disp(cameraRay)
    cameraRay4 = rotation4' * kInv4 * pp4;
    disp("The ray for vue4")
    disp(cameraRay4)
    % Calculate u3 and point
    u3 = u3getter(cameraRay, cameraRay4);
    [a,b,d] = triangulator(cameraRay,cameraRay4,u3,position(1:3),position4(1:3));
    answer = pcalculator(cameraRay, cameraRay4,a,b,position(1:3),position4(1:3));
    answer = -answer;
    disp("The triangulated 3d point is")
    disp(answer);
    disp('The error for this joint is')
    disp(distances(joint,frame))
    
    
    


    
end

    list = zeros(12,size(joints_new,1));
metrics = zeros(12, 5);
for a = 1:12
    for z = 1:size(joints_new,1)
        list(a,z) = euclidean_distance(joints_new(z,a,1:3), C(1:3,z,a));
    end
    % Calculate metrics
    metrics(a,1) = mean(list(a,:));
    metrics(a,2) = std(list(a,:));
    metrics(a,3) = min(list(a,:));
    metrics(a,4) = median(list(a,:));
    metrics(a,5) = max(list(a,:));
    
    
end
% Calculate metrics for all joints
all_joints_metrics = zeros(1, 5);
all_joints_metrics(1, 1) = mean2(list);
all_joints_metrics(1, 2) = std2(list);
all_joints_metrics(1, 3) = min(list(:));
all_joints_metrics(1, 4) = median(list(:));
all_joints_metrics(1, 5) = max(list(:));
    

% Calculate error sums per frame
frame_range = 1:size(joints_new);
frame_error_sums = zeros(1,size(joints_new,1));
for z = 1:size(joints_new,1)
    frame_error_sums(1, z) = sum(list(:,z));
end

figure();
plot(ids, frame_error_sums);
title("Sum of Errors for Time Sequence");
xlabel("Frame");
ylabel("Total Error");



profile report
profile off


% Necessary functions for camera projection

function[newpoint] = world2camera(Pmat, worldpoint)
    % Convert world coordinates to camera coordinates

    newpoint = Pmat * worldpoint;
end



function [pixelpoint] = camera2pixel(Kmat, point)
    % Convert film coordinates to pixel coordinates

    pixelpoint = Kmat*point;
    pixelpoint(1) = pixelpoint(1)/pixelpoint(3);
    pixelpoint(2) = pixelpoint(2)/pixelpoint(3);
    pixelpoint(3) = 1;
end

function [pixelpoint] = world2pixel(Pmat,worldpoint,Kmat)
camerapoint = world2camera(Pmat,worldpoint);
pixelpoint = camera2pixel(Kmat,camerapoint);
end


function [u3] = u3getter(cameraRay, cameraRay4)
    % Calculate u3 from camera rays
    u3 = cross(cameraRay,cameraRay4) / norm(cross(cameraRay,cameraRay4));
end


function [a,b,d] = triangulator(u1,u2,u3,c1,c2)
    uMatrix = [u1 u2 u3];
 
    c2minusc1 = c2-c1;
    

    vector = inv(uMatrix) * c2minusc1;
    a = vector(1);
    b = -vector(2);
    d = vector(3);

end


function [p] = pcalculator(u1,u2,a,b,c1,c2)
    p1 = c1 + a*u1;
    p2 = c2 + b*u2;
    p = (p1+p2)/2;
end

function displayframewithSkeleton(frame,B, ids)
filenamevue4mp4 = 'Subject4-Session3-24form-Full-Take4-Vue2.mp4';
vue2video = VideoReader(filenamevue4mp4);
mocapFnum=ids(frame);
vue2video.CurrentTime = (mocapFnum-1)*(50/100)/vue2video.FrameRate;
vid2Frame = readFrame(vue2video);
figure();

imagesc(vid2Frame)

hold on;
plot(B(1,frame,1), B(2,frame,1), "r.", "MarkerSize", 8)
plot(B(1,frame,2), B(2,frame,2), "r.", "MarkerSize", 8)
plot(B(1,frame,3), B(2,frame,3), "r.", "MarkerSize", 8)
plot(B(1,frame,4), B(2,frame,4), "r.", "MarkerSize", 8)
plot(B(1,frame,5), B(2,frame,5), "r.", "MarkerSize", 8)
plot(B(1,frame,6), B(2,frame,6), "r.", "MarkerSize", 8)
plot(B(1,frame,7), B(2,frame,7), "r.", "MarkerSize", 8)
plot(B(1,frame,8), B(2,frame,8), "r.", "MarkerSize", 8)
plot(B(1,frame,9), B(2,frame,9), "r.", "MarkerSize", 8)
plot(B(1,frame,10), B(2,frame,10), "r.", "MarkerSize", 8)
plot(B(1,frame,11), B(2,frame,11), "r.", "MarkerSize", 8)
plot(B(1,frame,12), B(2,frame,12), "r.", "MarkerSize", 8)
plot((B(1,frame,1)+B(1,frame,4))/2, (B(2,frame,1)+B(2,frame,4))/2, "r.", "MarkerSize", 10) %shoulder midpoint
plot((B(1,frame,7)+B(1,frame,10))/2, (B(2,frame,7)+B(2,frame,10))/2, "r.", "MarkerSize", 10) %hip midpoint
% lines from point to point
plot([B(1,frame,1);B(1,frame,2)], [B(2,frame,1); B(2,frame,2)], '-r'); %1->2
plot([B(1,frame,2);B(1,frame,3)], [B(2,frame,2); B(2,frame,3)], '-r'); %2->3
plot([B(1,frame,1);B(1,frame,4)], [B(2,frame,1); B(2,frame,4)], '-r'); %1->4
plot([B(1,frame,4);B(1,frame,5)], [B(2,frame,4); B(2,frame,5)], '-r'); %4->5
plot([B(1,frame,5);B(1,frame,6)], [B(2,frame,5); B(2,frame,6)], '-r'); %5->6
plot([B(1,frame,7);B(1,frame,8)], [B(2,frame,7); B(2,frame,8)], '-r'); %7->8
plot([B(1,frame,8);B(1,frame,9)], [B(2,frame,8); B(2,frame,9)], '-r'); %8->9
plot([B(1,frame,7);B(1,frame,10)], [B(2,frame,7); B(2,frame,10)], '-r'); %7->10
plot([B(1,frame,10);B(1,frame,11)], [B(2,frame,10); B(2,frame,11)], '-r'); %10->11
plot([B(1,frame,11);B(1,frame,12)], [B(2,frame,11); B(2,frame,12)], '-r'); %11->12
plot([(B(1,frame,1)+B(1,frame,4))/2;(B(1,frame,7)+B(1,frame,10))/2], [(B(2,frame,1)+B(2,frame,4))/2; (B(2,frame,7)+B(2,frame,10))/2], '-r'); % shoulder mid to hip mid
title(["Frame", ids(frame)]);
hold off;
end


function x = euclidean_distance(point1,point2)

x = sqrt(sum((point1(:) - point2(:)) .^ 2));
end

end
