function odomCallback(~, msg)
% ODOMCALLBACK Subscriber callback function for pose data    
%   ODOMCALLBACK(~,MSG) returns no arguments - it instead sets a 
%   global variable to the values of position and orientation that are
%   received in the ROS message MSG.

    % declare global variable to store position and orientation
    global odomPose
    
    % extract position and orientation from the ROS message and assign the
    % data to the global variable.
    euler = quat2eul([msg.Pose.Pose.Orientation.W, ...
                      msg.Pose.Pose.Orientation.X, ...
                      msg.Pose.Pose.Orientation.Y, ...
                      msg.Pose.Pose.Orientation.Z]);
    odomPose = [msg.Pose.Pose.Position.X, ...
                  msg.Pose.Pose.Position.Y, ...
                  euler(1)];
end