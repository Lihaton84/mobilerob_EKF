classdef ExtendedKalmanFilter < matlab.System
% EXTENDEDKALMANFILTER Localize robot with known landmarks.
%   EKF = ExtendedKalmanFilter('PropertyName', PropertyValue, ...) returns a
%   ExtendedKalmanFilter object with ech specified property set to the
%   specified value.
%
%   Step method syntax:
%
%   [ESTPOSE, ESTCOV] = step(EKF, POSE, ANGLES)
%
%   [ESTPOSE, ESTCOV] = step(EKF, POSE, ANGLES, RANGES)
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj, x) and y = obj(x) are
%   equivalent.
%
%   ExtendedKalmanFilter methods:
%
%   step  - Estimate robot pose using provided measurements
%   reset - Re-initialize the ExtendedKalmanFilter System object
%   plot  - Plot the filter estimates
%
%   ExtendedKalmanFilter properties:
%
%   sensorNoise        - Sensor noise [angle range] stddev
%   robotNoise         - Noise modifiers, [a1 ... a4] for the odometry model,
%                        [a1 ... a6] for the velocity model
%   initialPose        - Initial pose for localization
%   initialCovariance  - Covariance of initial pose
%   confidenceInterval - Confidence interval to use for plotting
%   nInputs            - Number of inputs to the step method, see step syntax
%   landmarkLocation   - Column major locations for the landmarks
%   motionModelType    - Type of the motion model to use, 'odometry' or
%                        'velocity'

    properties
        sensorNoise = [0.01 0.2] % [angle range]
        robotNoise = [0.1 0.1 0.1 0.1 0.1 0.1] % [a1 a2 a3 a4 a5 a6]
        confidenceInterval = 0.5;
        knownCorrespondes = false;
    end % properties

    properties (Nontunable)
        initialPose = [0 0 0]' % [x y theta]'
        initialCovariance = eye(3)
        nInputs = 3 % control, angle, range
        motionModelType = 'odometry' % 'odometry' or 'velocity'
        landmarkLocation = [5.0, 1.0, -5.5; % x_i
                            5.0, 2.5,  0.0] % y_i
    end % properties

    properties (Access = private)
        Pose
        Covariance
        previousOdomPose
        getControl
        getMeasurement
        measurementModel
        measurementModelJacobianWRTpose
        measurementCovariance
        motionModel
        motionModelJacobianWRTpose
        motionModelJacobianWRTnoise
        controlCovariance
       
    end % properties

    methods
        function obj = ExtendedKalmanFilter(varargin)
        % Constructor for the EKF object.
            setProperties(obj, nargin, varargin{:});
        end

        function handles = plot(obj, varargin)
        % PLOT Plot the filter estimates
            poseHandle = ...
                plot(obj.Pose(1), obj.Pose(2), 'o', ...
                     [obj.Pose(1), obj.Pose(1)+cos(obj.Pose(3))], ...
                     [obj.Pose(2), obj.Pose(2)+sin(obj.Pose(3))], ...
                     'color', 'k', 'markersize', 15, 'linewidth', 2);

            % Compute quantile for the desired percentile
            % There are three degrees of freedom [x y theta]
            k = sqrt(chi2inv(obj.confidenceInterval, 3));

            n = 100; % Number of points around ellipse
            p = 0:pi/n:2*pi; % angles around a circle

            % Compute eigen vector and values for the projection
            % on x-y-plane
            [eigvec, eigval] = eig(obj.Covariance(1:2, 1:2));
            % Compute transformation
            xy = [cos(p'), sin(p')] * sqrt(eigval) * eigvec';
            x = xy(:,1);
            y = xy(:,2);

            covarianceHandle = fill(obj.Pose(1)+k*x, obj.Pose(2)+k*y, 'k', ...
                                    'edgecolor', 'none', ...
                                    'facealpha', 0.3);

            handles = vertcat(poseHandle, covarianceHandle);

            % Set user properties if possible
            for i = 1:2:nargin-1
                for j = 1:length(handles)
                    if isprop(handles(j), varargin{i})
                        set(handles(j), varargin{i}, varargin{i+1});
                    end
                end
            end
        end
    end % methods

    methods (Access = protected)
        function resetImpl(obj)
        % RESETIMPL Reset to initial state
            obj.Pose = obj.initialPose;
            obj.Covariance = obj.initialCovariance;
            obj.previousOdomPose = obj.initialPose;
        end

        function setupImpl(obj, varargin)
        % SETUPIMPL Run one-time tasks to setup the object
            % Setup measurement model
            setupMeasurementModel(obj);
            % Set up motion model
            switch upper(obj.motionModelType)
                case 'ODOMETRY'
                    setupOdometryModel(obj);
                case 'VELOCITY'
                    setupVelocityModel(obj);
            end
        end

        function num = getNumInputsImpl(obj)
        % GETNUMINPUTSIMPL Return number of inputs
            num = obj.nInputs;
        end

        function validateInputsImpl(obj, varargin)
        % VALIDATEINPUTSIMPL Validate inputs to step at initialization
            obj.checkInputs(varargin{:});
        end

        function [estimatedPose, estimatedCovariance] = stepImpl(obj, varargin)
        % STEPIMPL Estimate robot pose
            % Get control and measurement
            control = obj.getControl(varargin{1});
            measurement = obj.getMeasurement(varargin{2:end});
            
            % Get the current estimate
            estimatedPose = obj.Pose;
            estimatedCovariance = obj.Covariance;

            % Prediction step
            [estimatedPose, estimatedCovariance] = obj.EKFpredict(...
                estimatedPose, estimatedCovariance, control);

            % Update step for each measurement
            location = obj.landmarkLocation;
            
            %There has to be a way to remove landmark association from measurement.
            %check what measurement holds inside
            
            if obj.knownCorrespondes == 'false'
                for landmark = 1:size(location,2) %goes through every landmark position
                    if ~isnan(measurement(1, landmark)) %check in what column the position is found

                        predZ = 0; % predicted measurements
                        predS = 0; % predicted measurement covariances
                        predH = 0; % predicted measurement Jacobians
                        minJ = inf;
                        Associated_xy = [0,0];
                        for i = 1:size(location,2)
                            x_dist = location(1, i) - obj.Pose(1);    %x difference how far robot is from feature x
                            y_dist = location(2, i) - obj.Pose(2);    %y difference how far robot is from feature y
                            q = x_dist^2 + y_dist^2; %euklidian distance from previous

                            predZ = [sqrt(q);wrapToPi(atan2(y_dist, x_dist) - obj.Pose(3))]; %calculate expected measurement for landmark (all landmarks are considered because of for loop

                            predH = [-x_dist/sqrt(q) -y_dist/sqrt(q) 0;... % calculate predicted measurement Jacobian
                                         y_dist/q        -x_dist/q      -1];

                            predCov = predH*obj.Covariance*predH' + obj.measurementCovariance; ...  % calculate predicted measurement covariance
                            
                            
                            z = mvnrnd(predZ,obj.measurementCovariance,50); %makind a random distribution of measurement size 50
                            
                            mahalf = mahal([measurement(1,landmark) measurement(2,landmark)],z);
                                                       
                            if minJ > mahalf 
                                minJ = mahalf;
                                Associated_xy = [location(1, i);location(2, i)];
                                DetectedLocation = i;
                            end
                            

                        end
                        
                      disp('detected feature number:');
                      disp(DetectedLocation);
                      
                         [estimatedPose, estimatedCovariance] = obj.EKFupdate(...
                          estimatedPose, estimatedCovariance, ...
                          measurement(:,landmark), Associated_xy); %location is given as a matrix [x,y]
                    end
                end
             

%             
%             % loop over all landmarks and compute MLE correspondence
%             predZ = zeros(n_landmarks, 1, 2); % predicted measurements
%             predS = zeros(n_landmarks, 2, 2); % predicted measurement covariances
%             predH = zeros(n_landmarks, 2, 3); % predicted measurement Jacobians
%             maxJ = 0; 
%             landmarkIndex = 0;
%             for j = 1:n_landmarks
%                 xDist = Landmark_Groundtruth(j, 2) - poseMeanBar(1);
%                 yDist = Landmark_Groundtruth(j, 3) - poseMeanBar(2);
%                 q = xDist^2 + yDist^2;
%                 
%                 % calculate predicted measurement
%                 predZ(j,:,:) = [sqrt(q);wrapToPi(atan2(yDist, xDist) - obj.Pose(3))];
%                         
%                 % calculate predicted measurement Jacobian
%                 predH(j,:,:) = [-xDist/sqrt(q) -yDist/sqrt(q) 0;
%                                 yDist/q        -xDist/q      -1];
%                             
%                 % calculate predicted measurement covariance
%                 predS(j,:,:) = squeeze(predH(j,:,:)) * poseCovBar ...
%                                * squeeze(predH(j,:,:))' + Q_t;
%                 
%                 % calculate probability of measurement correspondence          
%                 thisJ = det(2*pi*squeeze(predS(j,:,:)))^(-0.5)*exp(-0.5 * (z(:,k) - squeeze(predZ(j,:,:)))' ...
%                         * inv(squeeze(predS(j,:,:))) ...
%                         * (z(:,k) - squeeze(predZ(j,:,:))));
%                 
%                 % update correspondence if the probability is
%                 % higher than the previous maximum
%                 if thisJ > maxJ
%                     maxJ = thisJ;
%                     landmarkIndex = j;
%                 end
%              end
             else
                for landmark = 1:size(location,2) 
                    if ~isnan(measurement(1, landmark))
                        [estimatedPose, estimatedCovariance] = obj.EKFupdate(...
                            estimatedPose, estimatedCovariance, ...
                            measurement(:,landmark), location(:,landmark));
                    end
                end
           end  
            % Update internal state representation
            obj.Pose = estimatedPose;
            obj.Covariance = estimatedCovariance;
        end
    end % methods

    methods (Access = private)
        function setupMeasurementModel(obj)
        % SETUPMEASUREMENTMODEL Setup measurement model
            switch obj.getNumInputs()
                case 2 % only angle measurements
                    obj.measurementModel = @(pose,lm) ...
                        wrapToPi(atan2(lm(2)-pose(2),lm(1)-pose(1))-pose(3));
                    obj.measurementModelJacobianWRTpose = @(pose,lm) ...
                        [(lm(2)-pose(2))/((lm(1)-pose(1))^2+(lm(2)-pose(2))^2), ...
                         -(lm(1)-pose(1))/((lm(1)-pose(1))^2+(lm(2)-pose(2))^2), -1];
                    obj.measurementCovariance = obj.sensorNoise(1)^2;
                    obj.getMeasurement = @(varargin)varargin{1}';
                case 3 % angle + range measurements
                    obj.measurementModel = @(pose,lm) ...
                        [sqrt((lm(1)-pose(1))^2+(lm(2)-pose(2))^2);
                         wrapToPi(atan2(lm(2)-pose(2),lm(1)-pose(1))-pose(3))];
                    obj.measurementModelJacobianWRTpose = @(pose,lm) ...
                        [-(lm(1)-pose(1))/sqrt((lm(1)-pose(1))^2+(lm(2)-pose(2))^2), ...
                         -(lm(2)-pose(2))/sqrt((lm(1)-pose(1))^2+(lm(2)-pose(2))^2), 0;
                         (lm(2)-pose(2))/((lm(1)-pose(1))^2+(lm(2)-pose(2))^2), ...
                         -(lm(1)-pose(1))/((lm(1)-pose(1))^2+(lm(2)-pose(2))^2), -1];
                    obj.measurementCovariance = diag([obj.sensorNoise(2)^2, ...
                                                      obj.sensorNoise(1)^2]);
                    obj.getMeasurement = @(varargin)[varargin{2:-1:1}]';
            end
        end

        function setupOdometryModel(obj)
        % SETUPODOMETRYMODELS Setup odometry motion model
            obj.motionModel = @(pose,control) ...
                [pose(1)+control(2)*cos(control(1)+pose(3));
                 pose(2)+control(2)*sin(control(1)+pose(3));
                 wrapToPi(control(1)+control(3)+pose(3))];
            obj.motionModelJacobianWRTpose = @(pose,control) ...
                [1, 0,-control(2)*sin(control(1)+pose(3));
                 0, 1, control(2)*cos(control(1)+pose(3));
                 0, 0, 1];
            obj.motionModelJacobianWRTnoise = @(pose,control) ...
                [control(2)*sin(control(1)-pose(3)),-cos(control(1)-pose(3)), 0;
                 control(2)*cos(control(1)-pose(3)), sin(control(1)-pose(3)), 0;
                 -1, 0, -1];
            obj.controlCovariance = @(modifier,control) ...
                [modifier(1)*control(1)^2+modifier(2)*control(2)^2, 0, 0;
                 0, modifier(4)*(control(1)+control(3))^2+modifier(3)*control(2)^2, 0;
                 0, 0, modifier(1)*control(3)^2+modifier(2)*control(2)^2];
            obj.getControl = @obj.getOdometryControl;
        end

        function control = getOdometryControl(obj,odomPose)
        % GETODOMETRYCONTROL Calculate odometry control from input
            % Decompose relative odometry
            previousPose = obj.previousOdomPose;
            obj.previousOdomPose = odomPose;

            rotation1 = wrapToPi(atan2(odomPose(2) - previousPose(2), ...
                                 odomPose(1) - previousPose(1)) - previousPose(3));
            translation = sqrt((odomPose(1) - previousPose(1))^2 + ...
                               (odomPose(2) - previousPose(2))^2);
            rotation2 = wrapToPi(odomPose(3) - previousPose(3) - rotation1);
            control = [rotation1; translation; rotation2];
        end

        function setupVelocityModel(obj)
        % SETUPVELOCITYMODELS Setup velocity motion models
            obj.motionModel = @obj.velocityMotionModel;
            obj.motionModelJacobianWRTpose = @obj.velocityMotionModelJacobianWRTpose;
            obj.motionModelJacobianWRTnoise = @obj.velocityMotionModelJacobianWRTnoise;
            obj.controlCovariance = @(modifier,control) ...
                [modifier(1)*control(1)^2+modifier(2)*control(2)^2, 0, 0;
                 0, modifier(3)*control(1)^2+modifier(4)*control(2)^2, 0;
                 0, 0, modifier(5)*control(1)^2+modifier(6)*control(2)^2];
            obj.getControl = @(control)control;
        end

        function [pose, covariance] = EKFpredict(obj, pose, covariance, control)
        % EKFPREDICT Prediction step of the EKF filter
            G = obj.motionModelJacobianWRTpose(pose, control);
            V = obj.motionModelJacobianWRTnoise(pose, control);
            M = obj.controlCovariance(obj.robotNoise, control);
            covariance = G*covariance*G' + V*M*V';
            pose = obj.motionModel(pose, control);
        end

        function [pose, covariance] = EKFupdate(obj, pose, covariance, measurement, landmark)
        % EKFUPDATE Update step of the EKF filter
            innovation = measurement - obj.measurementModel(pose, landmark);
            H = obj.measurementModelJacobianWRTpose(pose, landmark);
            innovationCovariance = H*covariance*H' + obj.measurementCovariance;
            KalmanGain = covariance*H'/innovationCovariance;
            pose = pose + KalmanGain*innovation;
            covariance = covariance - KalmanGain*innovationCovariance*KalmanGain';
        end
    end % methods

    methods (Static, Access = private)
        function pose = velocityMotionModel(pose,control)
        % VELOCITYMOTIONMODEL Motion model for velocity control
            x = pose(1);
            y = pose(2);
            th = pose(3);
            v = control(1);
            w = control(2);
            dt = control(3);
            % Take into account the special case w=0
            if w==0
                dtv = dt*v;
                pose = [x+dtv*cos(th); y+dtv*sin(th); th];
            else
                r = v/w;
                th_dtw = th+dt*w;
                pose = [x+r*sin(th_dtw)-r*sin(th);
                        y-r*cos(th_dtw)+r*cos(th);
                        wrapToPi(th_dtw)];
            end
        end

        function jacobian = velocityMotionModelJacobianWRTpose(pose,control)
        % VELOCITYMOTIONMODELJACOBIANWRTPOSE Jacobian of the velocity motion
        % model w.r.t the pose of the robot
            th = pose(3);
            v = control(1);
            w = control(2);
            dt = control(3);
            % Take into account the special case w=0
            if w==0
                dtv = dt*v;
                jacobian = [1, 0,-dtv*sin(th);
                            0, 1, dtv*cos(th);
                            0, 0, 1];
            else
                r = v/w;
                th_dtw = th+dt*w;
                jacobian = [1, 0, r*cos(th_dtw)-r*cos(th);
                            0, 1, r*sin(th_dtw)-r*sin(th);
                            0, 0, 1];
            end
        end

        function jacobian = velocityMotionModelJacobianWRTnoise(pose,control)
        % VELOCITYMOTIONMODELJACOBIANWRTNOISE Jacobian of the velocity motion
        % model w.r.t. the noise in control inputs
            th = pose(3);
            v = control(1);
            w = control(2);
            dt = control(3);
            % Pre-compute some stuff
            sin_th = sin(th);
            cos_th = cos(th);
            % Take into account the special case w=0
            if w==0
                dt2v_2 = dt^2*v/2;
                jacobian = [dt*cos_th, -dt2v_2*sin_th, 0;
                            dt*sin_th, dt2v_2*cos_th, 0;
                            0, dt, dt];
            else
                r = v/w;
                drv = 1/w;
                drw = v/w^2;
                sin_th_dtw = sin(th+dt*w);
                cos_th_dtw = cos(th+dt*w);
                dtr = dt*r;
                jacobian = ...
                    [drv*sin_th_dtw-drv*sin_th, drw*sin_th-drw*sin_th_dtw+dtr*cos_th_dtw, 0;
                     drv*cos_th-drv*cos_th_dtw, drw*cos_th_dtw-drw*cos_th+dtr*sin_th_dtw, 0;
                     0, dt, dt];
            end
        end

        function checkInputs(varargin)
        % CHECKINPUTS Validate inputs
            control = varargin{1};
            validateattributes(control, {'double'}, ...
                {'vector', 'numel', 3, 'real', 'finite'}, 'step', 'control');
            angles = varargin{2};
            validateattributes(angles, {'double'}, ...
                {'vector', 'real'}, 'step', 'angles');

            if nargin == 4
                ranges = varargin{3};
                validateattributes(ranges, {'double'}, ...
                {'vector', 'real', 'nonnegative'}, 'step', 'ranges');

                if (numel(ranges) ~= numel(angles))
                    error('Input size mismatch');
                end
            end
        end
    end % methods
end % classdef
