clear all;
clc;

layout = jsondecode(fileread('Bunnybots_2023.json'));

table = [
    633.5,  14.5,  5.0, 135;
    148.5, 121.5,  9.0, 180;
     14.5, 309.5,  5.0, 315;
    499.5,  94.5,  9.0,   0;
    633.5,  14.5, 42.5, 135;
    148.5, 229.5,  9.0, 180;
     14.5, 309.5, 42.5, 315;
    484.5, 202.5,  9.0, 180;
    415.5, 320.0,  5.0, 270;
    163.5, 121.5,  9.0,   0;
    232.5,   4.0,  5.0,  90;
    484.5,  94.5,  9.0, 180;
    499.5, 202.5,  9.0,   0;
    163.5, 229.5,  9.0,   0];

for k=1:14
    tag(k).translation = table(k,1:3) * 2.54/100;
    tag(k).rollPitchYaw = [0, 0, table(k,4)] * pi/180;
end
%% limelight fmap


tagSizeMm = 6.5 * 25.4;
for k=1:numel(tag)
    T = eye(4);
    T(4,1:3) = tag(k).translation;
    
    theta = tag(k).rollPitchYaw(3);
    Rz = [cos(theta), -sin(theta), 0, 0;
          sin(theta),  cos(theta), 0, 0;
          0, 0, 1, 0;
          0, 0, 0, 1];
    
    X = Rz * T;

    
    fiducials(k).family = "apriltag3_36h11_classic";
    fiducials(k).id = k;
    fiducials(k).size = tagSizeMm;
    fiducials(k).transform = X(:);
    fiducials(k).unique = 1;
end

q.fiducials = fiducials;
limelight = jsonencode(q, 'PrettyPrint', true);
fid = fopen('Bunnybots_2023.fmap', 'wt');
fprintf(fid, '%s', limelight);
fclose(fid);