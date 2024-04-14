setenv('ROS_MASTER_URI','http://192.168.179.100:11311');
setenv('ROS_IP','192.168.179.38');
rosshutdown()
rosinit('http://192.168.179.100:11311','NodeHost','192.168.179.38')

%% Subscriber and publisher setup
vel_pub = rospublisher('/cmd_vel');
scansub = rossubscriber('/scan');
pos_sub = rossubscriber('/tf');

%% Controller setup
controller = controllerPurePursuit;
controller.DesiredLinearVelocity = 0.2;
controller.MaxAngularVelocity = 2;

%We found that the robot is relatively stable with a LookAheadDistance of
%0.4
controller.LookaheadDistance = 0.4;

%Since we're using the reference frame of the LIDAR position and angle are
%always 0
robotCurrentPose = [0; 0; 0];



while(1)

    %% Data retrieval
    scan = receive(scansub);
    cart = readCartesian(scan);

    hold on
    xlim([-4 4])
    ylim([-4 4])
   
    x = cart(:, 1);  % x-pos
    d = cart(:, 2);  % y-pos
    
    % Filter out points with y coordinates above 0 (to the right of the robot)
    filtered_indices = d <= 0;
    x = x(filtered_indices);
    d = d(filtered_indices);


    %% Fitting the line of the wall and plotting it
    
    mdl = fitlm(x,d);
    coef=mdl.Coefficients.Estimate;

    plot(x, (coef(1) + coef(2)*x), 'r')
    
    % Calculate the distance from the robot to the line to check if
    % distance is ~0.5 meter
    distance = abs(coef(1)) / sqrt(1 + coef(2)^2);
    
    fprintf('Closest distance from (0,0) to the line: %f\n', distance);

    %Defining a point to aim for 0.5 meters out from the wall and 1 meter
    %ahead
    aim_point = [1 0.5+(coef(2)*1+coef(1))];

    plot(aim_point(1),aim_point(2),'b.');

    %Setting the controller to go towards the aim-point
    controller.Waypoints = aim_point;
    [v, w] = controller(robotCurrentPose);
    update_vel(v,w,vel_pub)


end

function [true] = update_vel(v,w,vel_pub)

%Very simple function. We get a new linear and angular velocity from the
%controller and output it to the cmd_vel topic.
twistmsg = rosmessage(vel_pub);

twistmsg.Angular.Z = w;
twistmsg.Linear.X = v;

send(vel_pub,twistmsg);

end
