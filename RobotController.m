classdef RobotController < matlab.System
% ROBOTCONTROLLER Control a robot through ROS messages.
%   CONTROLLER = RobotController('PropertyName', PropertyValue, ...) returns a
%   RobotController object with ech specified property set to the specified
%   value.
%
%   Step method syntax:
%
%   step(CONTROLLER, POSE)
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj, x) and y = obj(x) are
%   equivalent.
%
%   RobotController methods:
%
%   step   - Publish robot control messages
%   reset  - Re-initialize the RobotController System object
%   isDone - End-of-data flag for reaching the final position
%
%   RobotController properties:
%
%   path               - A path for the robot to follow
%   lookaheadDistance  - Lookahead distance for both controllers
%   desiredVelocity    - Desired linear velocity for the pure pursuit control
%   maxAngularVelocity - Maximum angular velocity for the pure pursuit control
%   controllerType     - The type of controller to use, 'purePursuit' or
%                        'RezasController'

    properties
        path
        lookaheadDistance = 0.3
        desiredVelocity = 0.2
        maxAngularVelocity = pi
        goalTolerance = 0.1
        controllerType = 'purePursuit' % 'purePursuit' or 'RezasController'
    end % properties

    properties (Access = private)
        velPub
        velMsg
        controller
        runIndex = 1
        goalReached = false
    end % properties

    methods
        function obj = RobotController(varargin)
        % Constructor for the CONTROLLER object.
            setProperties(obj, nargin, varargin{:});
        end

        function status = isDone(obj)
        % ISDONE End-of-data flag for reaching the final position
            status = obj.goalReached;
        end
    end % methods

    methods (Access = protected)
        function resetImpl(obj)
        % RESETIMPL Reset to initial state
            obj.runIndex = 1;
            obj.goalReached = false;
        end

        function setupImpl(obj, varargin)
        % SETUPIMPL Run one-time tasks to setup the object
            % Create ROS publisher for sending out velocity commands.
            [obj.velPub, obj.velMsg] = rospublisher(...
                '/mobile_base/commands/velocity', ...
                'geometry_msgs/Twist');
            % Select the controller to use
            switch upper(obj.controllerType)
                case 'PUREPURSUIT'
                    obj.controller = @obj.purePursuit;
                case 'REZASCONTROLLER'
                    obj.controller = @obj.rezasController;
                otherwise
                    error('Unknown controller type.');
            end
        end

        function releaseImpl(obj)
        % RELEASEIMPL Release object resources
            delete(obj.velPub);
            obj.velPub = robotics.ros.Publisher.empty;
        end

        function validateInputsImpl(obj, pose)
        % VALIDATEINPUTSIMPL Validate inputs to step at initialization
            obj.checkInputs(pose);
        end

        function [v,w] = stepImpl(obj, pose)
        % STEPIMPL Publish robot control messages
            [v,w] = obj.controller(pose);

            obj.velMsg.Linear.X = v;
            obj.velMsg.Angular.Z = w;

            send(obj.velPub, obj.velMsg);
        end
    end % methods

    methods (Access = private)
        function [v,w] = purePursuit(obj, pose)
        % PUREPURSUIT Pure pursuit control
            pathPoses = obj.path.qd;
            pathLength = size(pathPoses, 2);
            lookahead = obj.lookaheadDistance;

            % Select first point in path that is at least the lookahead
            % distance away
            for idx = obj.runIndex:pathLength
                distanceToPoint = obj.distance(pose, pathPoses(:,idx));
                if distanceToPoint >= lookahead
                    % Transform the point to robot coordinate frame
                    bearingToPoint = obj.bearing(pose, pathPoses(:,idx));
                    [~, dy] = pol2cart(bearingToPoint, distanceToPoint);
                    break
                end
            end
            obj.runIndex = idx;

            % Check if approaching the final point in path
            if idx >= pathLength && ~obj.goalReached
                % Transform the goal to robot coordinate frame
                distanceToGoal = obj.distance(pose, pathPoses(:,end));
                bearingToGoal = obj.bearing(pose, pathPoses(:,end));
                [dx, dy] = pol2cart(bearingToGoal, distanceToGoal);
                % If closer than the specified tolerance, set
                % the goal as reached
                if abs(dx) < obj.goalTolerance
                    obj.goalReached = true;
                end
            end

            % If the goal hasn't been reached, calculate the angular velocity
            % so as to approach the lookahead point along an arc with the
            % radius of v/w
            if ~obj.goalReached
                v = obj.desiredVelocity;
                w = sign(dy)*min(v*2*abs(dy)/lookahead^2, obj.maxAngularVelocity);
            else
                v = 0;
                w = 0;
            end
        end

        function [vc,wc] = rezasController(obj, pose)
            [dx,dy] = pol2cart(pose(3),obj.lookaheadDistance);
            q = [pose(1)+dx; pose(2)+dy];
            psiB = pose(3);

            % find the closest point
            % search closest point near current runIndex
            nNeighbor = 2;
            sIndex = obj.runIndex-nNeighbor;
            eIndex = obj.runIndex+nNeighbor;
            EIndex = length(obj.path.time);
            Err = zeros(1,2*nNeighbor+1);
            if sIndex<=0
                sIndex = 1;
                eIndex = 2*nNeighbor+1;
            end

            if eIndex>EIndex
                sIndex = EIndex-(2*nNeighbor+1);
                eIndex = EIndex;
            end
            for i = sIndex:eIndex
                Err(i-sIndex+1) = norm(q - obj.path.qd(:, i));
            end
            % update runIndex
            [~,i] = min(Err);
            obj.runIndex = i+sIndex-1;

            if obj.runIndex == length(obj.path.qd)
                obj.goalReached = true;
                vc = 0;
                wc = 0;
                return
            end

            % error vector
            sampleT = obj.path.time(2)-obj.path.time(1);
            P = obj.path.qd(:, obj.runIndex);
            Pp = obj.path.qd_dot(:, obj.runIndex);
            Pp_del = obj.path.qd_dot(:, obj.runIndex+1);

            nPp = sqrt(Pp'*Pp);
            T = Pp/nPp; % tangent vector
            N = [-T(2); T(1)]; % normal vector

            nPp_del = sqrt(Pp_del'*Pp_del);
            T_del = Pp_del/nPp_del;
            Tp = (T_del-T)/sampleT; % derivative of the tangent vector
            Cc = Tp'*N/nPp; % curvature
            if abs(Cc) >5
                Cc = 0;
            end

            psiT = atan2(Pp(2),Pp(1));
            cpsiT=cos(psiT); spsiT=sin(psiT);

            RpsiT=[cpsiT   spsiT;
                   -spsiT  cpsiT];

            psi_e = psiB-psiT; psi_e = wrapToPi(psi_e);

            pe = RpsiT*(q-P); x_e = pe(1); y_e= pe(2);

            % control signals
            vc = norm(obj.path.qd_dot(obj.runIndex));
            s_dot = vc*cos(psi_e)/(1-y_e*Cc);
            %PFmethod = 'PF_time';
            PFmethod = 'PF_length';
            %PFmethod = 'TT';

            switch PFmethod
                case 'PF_time'
                lambda0 = 0.5;
                k3 = 2*lambda0;
                k2 = lambda0*lambda0/min(0.1,abs(vc));
                w1 = -k2*y_e - 2*k3*psi_e;
                wc = w1 + Cc*s_dot;
               case 'PF_length'
                % normalize for v
                lambda0 = 3;
                k3 = 2*lambda0;
                k2 = lambda0*lambda0;
                w1 = -k2*y_e - k3*psi_e;
                wc = w1*abs(vc) + Cc*s_dot;
                case 'TT'
%             % Trajectory tracking
% variables need to be defined base on time not s
%             cpsiB=cos(psiB); spsiB=sin(psiB);
%             RpsiB=[cpsiB   spsiB;
%                    -spsiB  cpsiB];
%
%             pe = RpsiB*(q-P); x_e = pe(1); y_e= pe(2);
%             vd = norm(obj.path.qd_dot(obj.runIndex));
%             k1 = 2*a; k3 = k1;
%             k2 = (a*a-wd*wd)/min(0.1, abs(vd)); % make sure this will work for reversing, that is, vd<0
%
%             u1 = -k1*x_e;
%             u2 = -k2*y_e - k3*psi_e;
%
%             vc = vd*cos(psi_e) - u1;
%             wc = wd - u2;
            otherwise
               vc=0;
               wc=0;
            end
        end

    end % methods

    methods (Static, Access = private)
        function b = bearing(pose1, pose2)
        % BEARING Bearing from pose1 to pose2
            b = wrapToPi(atan2(pose2(2)-pose1(2), pose2(1)-pose1(1)) - pose1(3));
        end

        function d = distance(pose1, pose2)
        % DISTANCE Euclidean distance between poses
            d = sqrt((pose2(1)-pose1(1))^2 + (pose2(2)-pose1(2))^2);
        end

        function checkInputs(pose)
        % CHECKINPUTS Validate inputs
            validateattributes(pose, {'double'}, ...
                {'vector', 'numel', 3, 'real', 'finite'}, 'step', 'pose');
        end
    end % methods
end % classdef
