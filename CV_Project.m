%% Calibration Parameters
% Left RGB Camera Intrinsic Parameters
fx_rgb_left = 517.57040; %546.13865;
fy_rgb_left = 518.73475; %535.59453; 
cx_rgb_left = 320; %264.48866; %
cy_rgb_left = 240; %220.24823; %

% Right RGB Camera Intrinsic Parameters
fx_rgb_right = 520.08735; %516.06233; 
fy_rgb_right = 509.26543; %509.59230;
cx_rgb_right = 320; %259.49525; %
cy_rgb_right = 240; %234.21632 %274.55578; 

% Kinect Depth camera parameters
fx_d =  5.7616540758591043e+02;
fy_d = 5.7375619782082447e+02;
cx_d = 3.2442516903961865e+02;
cy_d = 2.3584766381177013e+02;

% Extrinsic parameters between RGB and Depth camera for Kinect V1
% Rotation matrix
R =  inv([  9.9998579449446667e-01, 3.4203777687649762e-03, -4.0880099301915437e-03;
    -3.4291385577729263e-03, 9.9999183503355726e-01, -2.1379604698021303e-03;
    4.0806639192662465e-03, 2.1519484514690057e-03,  9.9998935859330040e-01]);

% Translation vector.
T = -[  2.2142187053089738e-02, -1.4391632009665779e-04, -7.9356552371601212e-03 ]';

%% Load last pair of images for 3D reconstruction later
% Load Left RGB and Depth scene image
left_rgb_img = imread('LeftRgbobjects_scene21.png');
left_depth_img = imread('LeftDepthobjects_scene21.png');

% Load Right RGB and Depth scene image
right_rgb_img = imread('RightRgbobjects_scene21.png');
right_depth_img = imread('RightDedpthobjects_scene21.png');

% Lets have a look at the images
figure();
subplot(1,2,1) 
imshow(left_rgb_img );
subplot(1,2,2) 
imshow(left_depth_img,[0.8,3.0],'Colormap', jet(255));
figure();
subplot(1,2,1) 
imshow(right_rgb_img);
subplot(1,2,2) 
imshow(right_depth_img,[0.8,3.0],'Colormap', jet(255));
%% Step1: Back-project every 2D point from depth image into 3D space
% Left Camera:
[n_row, n_col] = size(left_depth_img);
left_depth_map = zeros(n_row, n_col, 3);  
for x = 1:n_row
    for y = 1:n_col
        % Back-project every 2D point from depth image into 3D space
        left_depth_map(x, y, 1) = (y - cx_d) * double(left_depth_img(x, y)) / fx_d;
        left_depth_map(x, y, 2) = (x - cy_d) * double(left_depth_img(x, y)) / fy_d;
        left_depth_map(x, y, 3) = double(left_depth_img(x, y));
       
    end
end

% Right Camera:
[n_row, n_col] = size(right_depth_img);
right_depth_map = zeros(n_row, n_col, 3);
for x = 1:n_row
    for y = 1:n_col
        % Back-project every 2D point from depth image into 3D space
        right_depth_map(x, y, 1) = (y - cx_d) * double(right_depth_img(x, y)) / fx_d;
        right_depth_map(x, y, 2) = (x - cy_d) * double(right_depth_img(x, y)) / fy_d;
        right_depth_map(x, y, 3) = double(right_depth_img(x, y));
    end
end


%% Step 2: Project every transformed 3D point into the RGB image
% Left Camera:
index = 0;
pc_rbg_left = zeros(n_row * n_col, 3);
pc_rbg_left_col = zeros(n_row * n_col, 3);
for x = 1:n_row
    for y = 1:n_col
        if left_depth_map(x, y, 3) > 0
            % Apply transformation between Depth and RGB camera
            XYZ_depth = [left_depth_map(x, y, 1), left_depth_map(x, y, 2), left_depth_map(x, y, 3)];
            XYZ_rgb = R * XYZ_depth' + T;
            % Project every transformed 3D point into the RGB image
            x_rgb = round(fx_rgb_left * XYZ_rgb(1) / XYZ_rgb(3) + cx_rgb_left);
            y_rgb = round(fy_rgb_left * XYZ_rgb(2) / XYZ_rgb(3) + cy_rgb_left);

            if x_rgb > 0 && y_rgb > 0 && x_rgb < n_col  && y_rgb < n_row
                % look up color corrsponding to a given 3D point
                color = left_rgb_img(y_rgb, x_rgb, :);
                index = index + 1;
                % create new point cloud with color
                pc_rbg_left(index,:) =  XYZ_rgb;
                pc_rbg_left_col(index, :) = color;
            end
        end
    end
end
pc_rbg_left = pc_rbg_left(1:index,:);
pc_rbg_left_col = pc_rbg_left_col (1:index,:);

figure()

pcshow(left_depth_map)
figure()
pcshow(pc_rbg_left, pc_rbg_left_col / 256.0)


% Right camera:
index = 0;
pc_rbg_right = zeros(n_row * n_col, 3);
pc_rbg_right_col = zeros(n_row * n_col, 3);
for x = 1:n_row
    for y = 1:n_col
         if right_depth_map(x, y, 3) > 0
            % Apply transformation between Depth and RGB camera
            XYZ_depth = [right_depth_map(x, y, 1), right_depth_map(x, y, 2), right_depth_map(x, y, 3)];
            XYZ_rgb = R * XYZ_depth' + T;
            % Project every transformed 3D point into the RGB image
            x_rgb = round(fx_rgb_right * XYZ_rgb(1) / XYZ_rgb(3) + cx_rgb_right);
            y_rgb = round(fy_rgb_right * XYZ_rgb(2) / XYZ_rgb(3) + cy_rgb_right);

            if x_rgb > 0 && y_rgb > 0 && x_rgb < n_col  && y_rgb < n_row
                % look up color corrsponding to a given 3D point
                color = right_rgb_img(y_rgb, x_rgb, :);
                index = index + 1;
                % create new point cloud with color
                pc_rbg_right(index,:) =  XYZ_rgb;
                pc_rbg_right_col(index, :) = color;
            end
        end
    end
end
pc_rbg_right = pc_rbg_right(1:index,:);
pc_rbg_right_col = pc_rbg_right_col (1:index,:);

figure()
 
pcshow(right_depth_map)
figure()
pcshow(pc_rbg_right, pc_rbg_right_col / 256.0)


%% Step3: Transformation from left 3D coordinate to right 3D coordinate camera frame

% Transformation vectors between left and right RBG cameras
T_rbg = [ -954.57277, 99.75455, 462.56569] - [66.49332, 16.62784, 128.72005*0.5]; %[-1034.35109, 66.12586, 410.88503] * 0.9 ;
R_rbg = [ -0.03190 0.73381 -0.15668 ]; %[ -0.01157 0.68566 -0.10888 ] ;
R_rbg = rotationVectorToMatrix(R_rbg);
R_rbg = inv(R_rbg);

% Calculate corresponding XYZ translated from left to right
len = length(pc_rbg_left);
pc_rbg_left_to_right = zeros(len, 3);
for i=1:len
    pc_rbg_left_to_right(i, :) = (R_rbg *  pc_rbg_left(i, :)' + T_rbg')';
end

% Merge point cloud 3D from both left and right cameras:
figure()
pcshow([pc_rbg_left_to_right; pc_rbg_right], [pc_rbg_left_col/256.0; pc_rbg_right_col/256])
