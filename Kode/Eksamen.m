addpath('eksamen_includes.m');

setenv('ROS_MASTER_URI','http://192.168.252.100:11311');

setenv('ROS_IP','192.168.252.38');

rosshutdown()
rosinit('http://192.168.252.100:11311','NodeHost','192.168.252.38')

%% Subscriber and publisher setup
cam_sub = rossubscriber('/usb_cam/image_raw/compressed',"DataFormat", "Struct");
vel_pub = rospublisher('/cmd_vel');
scansub = rossubscriber('/scan');
pos_sub = rossubscriber('/tf');

reset_pub = rospublisher('/reset');
send(reset_pub,rosmessage(reset_pub));

%% Controller setup
controller = controllerPurePursuit;
controller.DesiredLinearVelocity = 0.2;
controller.MaxAngularVelocity = 2;

%We found that the robot is relatively stable with a LookAheadDistance of
%0.4
controller.LookaheadDistance = 0.4;

%Since we're using the reference frame of the LIDAR position and angle are
%always 0

first_PRM = createFirstPRM();
second_PRM = createSecondPRM();

ZONE_B_X_VALUE = 5;

state = 1   ;
drive_distance = 0.5;


while (1)
    switch(state)

        case 1
            follow_road_map(pos_sub,vel_pub,first_PRM);
            state = 2;
        
        case 2
            follow_wall(drive_distance, scansub, vel_pub, pos_sub, controller);
            [circle_seen, center] = look_for_circle(pos_sub,vel_pub,cam_sub);
            if (circle_seen == 1)
                    drive_distance = 0.2;
                elseif (circle_seen == 2)
                    state = 3;
                    drive_distance = 0.5;
                else
                    drive_distance = 0.5;
            end



        case 3
            drive_to_circle_and_take_picture(cam_sub,vel_pub,scansub);
            state = 4;


        case 4
            follow_road_map(pos_sub,vel_pub,second_PRM);

            state = 5;


        case 5
           
                follow_wall(drive_distance, scansub, vel_pub, pos_sub, controller);
                circle_seen = look_for_circle(pos_sub,vel_pub,cam_sub);

                % if (circle_seen == 1)
                %     drive_distance = 0.2;
                % elseif (circle_seen == 2)
                %     state = 3;
                %     drive_distance = 0.5;
                % else
                %     drive_distance = 0.5;
                % end
            state = 6;


        case 6
            drive_to_circle_and_take_picture(cam_sub,vel_pub,scansub);


    end
    disp("pik")
end







function [true] = follow_wall(drive_distance, scansub, vel_pub, pos_sub, controller)
    start_pos = update_pos(pos_sub);
    robotCurrentPose = update_pos(pos_sub);


    while(norm(robotCurrentPose(1:2) - start_pos(1:2)) < drive_distance )
        
        %% Data retrieval
        scan = receive(scansub);
        cart = readCartesian(scan);
    
        hold on
        xlim([-3 3])
        ylim([-3 3])
       
        x = cart(:, 1);  % x-pos
        d = cart(:, 2);  % y-pos
        
        % Filter out points with y coordinates above 0 (to the right of the robot)
        filtered_indices = d >= 0;
        x = x(filtered_indices);
        d = d(filtered_indices);
    
    
        %% Fitting the line of the wall and plotting it
        
        mdl = fitlm(x,d);
        coef=mdl.Coefficients.Estimate;
    
        plot(x, (coef(1) + coef(2)*x), 'r')
        
        % Calculate the distance from the robot to the line to check if
        % distance is ~0.5 meter
        distance = abs(coef(1)) / sqrt(1 + coef(2)^2);
        
        % fprintf('Closest distance from (0,0) to the line: %f\n', distance);
    
        %Defining a point to aim for 0.5 meters out from the wall and 1 meter
        %ahead
        aim_point = [1 (coef(2)*1+coef(1))-1];
    
        plot(aim_point(1),aim_point(2),'bx');
    
        %Setting the controller to go towards the aim-point
        controller.Waypoints = aim_point;
        [v, w] = controller([0 0 0]);
        update_vel(v,w,vel_pub)

        robotCurrentPose = update_pos(pos_sub);


    end

end

function [seen, center] = look_for_circle(pos_sub,vel_pub,cam_sub)
    start_pos = update_pos(pos_sub);
    now_pos = start_pos;
    while(abs((now_pos(3)-start_pos(3)))<1.5)
        update_vel(0,1,vel_pub);
        now_pos = update_pos(pos_sub);
        % disp(abs((now_pos(3)-start_pos(3))))
    end

    update_vel(0,0,vel_pub);
    % input_image = imrotate(rosReadImage(imgraw),180,"bilinear");
    [seen, center] = detectRedCircle(cam_sub);
    
    if seen ~= 2
        while((now_pos(3)-start_pos(3))>0.5)
            update_vel(0,-1,vel_pub);
            now_pos = update_pos(pos_sub);
            % disp(abs((now_pos(3)-start_pos(3))))
        end
    end


end

function [path] = createFirstPRM()

    map = binaryOccupancyMap(29.3,28.3,10);

    walls = zeros(40,60);

    % Left wall
    walls(1:283,1) = 1;
    % Right wall
    walls(1:283,293) = 1;
    % Top wall
    walls(1,1:293) = 1;
    % Bottom wall
    walls(283,1:293) =1;

    
    % % Zone a and rooms next to
    walls(1:230,59:293) = 1;
    
    % Door before zone b
    walls(59:98,1:32) = 1;
    
    % Rooms next to zone c and corridor
    walls(240:283,59:249) = 1;

    setOccupancy(map,[1,1],walls,'grid')
    inflate(map, 0.2);
    PRM = mobileRobotPRM(map,500);
    startLocation = [5.5 25];
    endLocation = [5.5 21];
    path = findpath(PRM,startLocation,endLocation);
    figure
    show(PRM)

end

function [path] = createSecondPRM()

    map = binaryOccupancyMap(29.3,28.3,10);

    walls = zeros(40,60);

    % Left wall
    walls(1:283,1) = 1;
    % Right wall
    walls(1:283,293) = 1;
    % Top wall
    walls(1,1:293) = 1;
    % Bottom wall
    walls(283,1:293) =1;

    
    % % Zone a and rooms next to
    walls(1:230,59:293) = 1;
    
    % Door before zone b
    walls(59:98,1:32) = 1;
    
    % Rooms next to zone c and corridor
    walls(240:283,59:249) = 1;

    setOccupancy(map,[1,1],walls,'grid')
    inflate(map, 0.2);
    PRM = mobileRobotPRM(map,500);
    startLocation = [5.5 25];
    endLocation = [25.5 3.5];
    path = findpath(PRM,startLocation,endLocation);
    figure
    show(PRM)

end


function [true] = follow_road_map(pos_sub,vel_pub,path)
    %% Controller setup
    controller = controllerPurePursuit;
    controller.DesiredLinearVelocity = 0.2;
    controller.MaxAngularVelocity = 2;
    
    %We found that the robot is relatively stable with a LookAheadDistance of
    %0.4
    controller.LookaheadDistance = 0.4;
    goalRadius = 0.5;
    robotCurrentPose = update_pos(pos_sub);
    distanceToGoal = norm(robotCurrentPose - path(end, :))

    controller.Waypoints = path;

    while(distanceToGoal > goalRadius)   
    
        %We start the loop by updating the position of the robot
        Pose = update_pos(pos_sub);

        robotCurrentPose = [Pose(1)+5.5; Pose(2)+25; Pose(3)]
    
        % Compute the controller outputs, i.e., the inputs to the robot
        [v, w] = controller(robotCurrentPose);
    
        %Then we update the angular and linear velocities based on the
        %controller outputs.
        update_vel(v,w,vel_pub)
    
        %At last we check the distance to goal for whether we have reached the
        %end of the path.
        distanceToGoal = norm(robotCurrentPose(1:2) - path(end, :)')
    
    end
end

function [true] = drive_to_circle_and_take_picture(cam_sub,vel_pub,scansub)
    [found, center] = detectRedCircle(cam_sub);
    % disp(center)
    distanceToWall = 1;

     while(center>320)
        [found, center] = detectRedCircle(cam_sub);
        % disp(center)
        distanceToWall = 1;
        update_vel(0,0.1,vel_pub);
     end
        
     while(center<320)
        [found, center] = detectRedCircle(cam_sub);
        % disp(center)
        distanceToWall = 1;
        update_vel(0,0.1,vel_pub);
        update_vel(0,-0.1,vel_pub);
     end
    
    while(distanceToWall > 0.5)
        [found, center] = detectRedCircle(cam_sub);

        if(center>320)
            update_vel(0.1,0.05,vel_pub);
        else
            update_vel(0.1,-0.05,vel_pub);
        end
        %% Data retrieval
        scan = receive(scansub);
        cart = readCartesian(scan);
    
        hold on
        xlim([-3 3])
        ylim([-3 3])
       
        x = cart(:, 1);  % x-pos
        d = cart(:, 2);  % y-pos
        
        % Filter out points with x coordinates above 0 (to the front of the robot)
        filtered_indices = x >= 0;
        x = x(filtered_indices);
        d = d(filtered_indices);
    
    
        %% Fitting the line of the wall and plotting it
        
        mdl = fitlm(x,d);
        coef=mdl.Coefficients.Estimate;
        
        % Calculate the distance from the robot to the line to check if
        % distance is ~0.5 meter
        distanceToWall = abs(coef(1)) / sqrt(1 + coef(2)^2);
        disp(distanceToWall)
       
    end
end



function [true] = update_vel(v,w,vel_pub)

    %Very simple function. We get a new linear and angular velocity from the
    %controller and output it to the cmd_vel topic.
    twistmsg = rosmessage(vel_pub);
    
    twistmsg.Angular.Z = w;
    twistmsg.Linear.X = v;
    
    send(vel_pub,twistmsg);

end


function [output] = update_pos(pos_sub)
    
    %We start by getting the transform data from the tf topic
    receive(pos_sub,10);
    
    %the x and y data is = to the x and y of the tf topic
    pos.x = pos_sub.LatestMessage.Transforms.Transform.Translation.X;
    pos.y = pos_sub.LatestMessage.Transforms.Transform.Translation.Y;
    
    %We use the function quat2angle to transform the angle from quaternion to
    %yaw in radians
    quat.X = pos_sub.LatestMessage.Transforms.Transform.Rotation.X;
    quat.Y = pos_sub.LatestMessage.Transforms.Transform.Rotation.Y;
    quat.Z = pos_sub.LatestMessage.Transforms.Transform.Rotation.Z;
    quat.W = pos_sub.LatestMessage.Transforms.Transform.Rotation.W;
    
    [roll,pitch,yaw] = quat2angle([quat.X,quat.Y,quat.Z,quat.W]);
    
    pos.theta = yaw;
    
    %In the end we output the data and add the initial position of the robot to
    %x and y
    output = [pos.x; pos.y; pos.theta];

end


function [circleFound, circleColor, confidence] = detectColoredCircle(img)
    % Konverter billedet til gråskala
    grayImg = rgb2gray(img);

    % Brug morphologisk erosion og derefter dilation til at forbedre billedbehandling, valgfrit
    se = offsetstrel('ball',5,5);
    imgProcessed = imdilate(imerode(grayImg, se), se);

    % Find cirkler i billedet
    [centers, radii, metric] = imfindcircles(imgProcessed, [10 100], 'ObjectPolarity', 'dark', 'Sensitivity', 0.85);

    % Initialiser output variabler
    circleFound = false;
    circleColor = '';
    confidence = 0;

    % Tjek om der blev fundet nogen cirkler
    if ~isempty(centers)
        circleFound = true;
        confidence = max(metric);  % Maksimale confidence værdi fra detektionerne
        
        % Antag at den cirkel med højeste confidence er den vi er interesseret i
        bestCircleIndex = find(metric == max(metric), 1);
        bestCircleColor = impixel(img, centers(bestCircleIndex,1), centers(bestCircleIndex,2));
        
        % Definer tærskler for rød og lilla farve
        redThresholdLow = [150, 0, 0];
        redThresholdHigh = [255, 100, 100];
        purpleThresholdLow = [75, 0, 75];
        purpleThresholdHigh = [140, 80, 150];

        % Tjek farve inden for den bedste cirkel
        if all(bestCircleColor(1,:) >= redThresholdLow) & all(bestCircleColor(1,:) <= redThresholdHigh)
            circleColor = 'Red';
        elseif all(bestCircleColor(1,:) >= purpleThresholdLow) & all(bestCircleColor(1,:) <= purpleThresholdHigh)
            circleColor = 'Purple';
        else
            circleColor = 'Other';
        end
    end

    % Visualisér detektionen
    imshow(img);
    hold on;
    viscircles(centers, radii, 'EdgeColor', 'b');
    hold off;
    title(sprintf('Detected Circle: %s (%.2f%% Confidence)', circleColor, confidence * 100));

end

function [found, center] = detectRedCircle(cam_sub)
    imgraw = receive(cam_sub); % a serialized compressed image
    
    input_image = imrotate(rosReadImage(imgraw),180,"bilinear");
    
    imwrite(input_image, 'test_image.jpg');
    
    % input_image = imread("test_image.jpg");
    
    % Convert the image from RGB to HSV color space
    hsv_image = rgb2hsv(input_image);
    
    % Define thresholds for the hue, saturation, and value channels
    hue_threshold = [0 0.1]; % Values for red hues
    saturation_threshold = [0.5 1]; % Minimum saturation to avoid grays/blacks
    value_threshold = [0.5 1]; % Full range of value
    
    % Create a mask using the threshold values
    hue_mask = (hsv_image(:,:,1) >= hue_threshold(1)) & (hsv_image(:,:,1) <= hue_threshold(2));
    saturation_mask = (hsv_image(:,:,2) >= saturation_threshold(1)) & (hsv_image(:,:,2) <= saturation_threshold(2));
    value_mask = (hsv_image(:,:,3) >= value_threshold(1)) & (hsv_image(:,:,3) <= value_threshold(2));
    
    % Combine the masks
    final_mask = hue_mask & saturation_mask & value_mask;
    
    % Apply the mask to the original image
    output_image = input_image;
    output_image(repmat(~final_mask,[1 1 3])) = 0; % Set non-masked pixels to black
    
    SE = strel('disk',5)
    
    output_image = imerode(output_image,SE);
    output_image = imdilate(output_image,SE);
    
    
    gray_image = rgb2gray(output_image);
    
    output_image = gray_image > 0;
    
    [centers,radii,confidence] = imfindcircles(output_image,[10 200]);
    
    % display(centers)
    % display(radii)
    % display(confidence)
    
    
    % Display the original and thresholded images
    % figure;
    subplot(1,2,1);
    imshow(input_image);
    title('Original Image');

    if ~isempty(confidence)
        center = centers(1);
        if(confidence(1)>0.1)
            % centers = centers(1);
            % radii = radii(1);
            viscircles(centers(1, :), radii(1),'EdgeColor','b'); 
            found = 1;
        end
        
        if(confidence(1)>0.3)
            % centers = centers(1);
            % radii = radii(1);
            found = 2;
        end
    else
        found = 0;
        center = [0 0];
    end
    
    
    % viscircles(centers, radii,'EdgeColor','b'); 
    hold on;
    subplot(1,2,2);
    imshow(output_image);
    title('Thresholded Image');

end


