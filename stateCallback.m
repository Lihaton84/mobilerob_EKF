function stateCallback(~, msg)
% STATECALLBACK Subscriber callback function for Gazebo state data
%   STATECALLBACK(~,MSG) returns no arguments - it instead sets a
%   global variable to the values of position and orientation that are
%   received in the ROS message MSG.

    % declare global variable to store position and orientation
    global gazeboPose

    poseMsg = msg.Pose(strcmp(msg.Name, 'mobile_base'));
    quat = [poseMsg.Orientation.W, poseMsg.Orientation.X, ...
            poseMsg.Orientation.Y, poseMsg.Orientation.Z];
    rotation = quat2eul(quat);
    gazeboPose = [poseMsg.Position.X, ...
                  poseMsg.Position.Y, ...
                  rotation(1)];
end