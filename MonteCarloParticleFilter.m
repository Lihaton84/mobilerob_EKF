classdef MonteCarloParticleFilter < matlab.System
% MONTECARLOPARTICLEFILTER Localize robot with known landmarks.
%   MCL = MonteCarloParticleFilter('PropertyName', PropertyValue, ...) returns a
%   MonteCarloParticleFilter object with each specified property set to the
%   specified value.
%
%   Step method syntax:
%
%   [ESTPOSE, ESTCOV] = step(MCL, CONTROL, ANGLES)
%
%   [ESTPOSE, ESTCOV] = step(MCL, CONTROL, ANGLES, RANGES)
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj, x) and y = obj(x) are
%   equivalent.
%
%   MonteCarloParticleFilter methods:
%
%   step  - Estimate robot pose using provided measurements
%   reset - Re-initialize the MonteCarloParticleFilter System object
%   plot  - Plot the particles and mean value
%
%   MonteCarloParticleFilter properties:
%
%   sensorNoise                - Sensor noise [angle range] stddev
%   robotNoise                 - Noise modifiers [a1 ... a4] for the odometry model
%                                [a1 ... a6] for the velocity model
%   resamplingInterval         - Number of filter updates between resampling the particles.
%                                Set to 0 to disable.
%   effectiveParticleThreshold - Threshold for number of effective particles in percentage
%                                of nParticles before resampling. Set to 0 to disable.
%   initialPose                - Initial pose for localization
%   initialCovariance          - Covariance of initial pose
%   nInputs                    - Number of inputs to the step method
%   nParticles                 - Number of particles to use
%   landmarkLocation           - Column major locations for the landmarks
%   useAugmentation            - Whether to augment the particles with random samples
%   augmentationProbability    - Probability for random augmentation
%   xlim                       - Limits for random augmentation
%   ylim                       - Limits for random augmentation
%   motionModelType            - Type of the motion model to use, 'odometry' or
%                                'velocity'

    properties
        sensorNoise = [0.02 0.3] % [angle range]
        robotNoise = [0.01 0.01 0.01 0.02 0.01 0.01]% [a1 a2 a3 a4 a5 a6]
        resamplingInterval = 10
        effectiveParticleThreshold = 0.1
    end % properties

    properties (Nontunable)
        initialPose = [0 0 0]' % [x y theta]
        initialCovariance = 0.1*eye(3)
        nInputs = 3
        nParticles = 500
        useAugmentation = false
        augmentationProbability = 0.01
        xlim = [-6 6]
        ylim = [-2 5]
        motionModelType = 'odometry' % 'odometry' or 'velocity'
        landmarkLocation = [5.0, 1.0, -5.5; % x_i
                            5.0, 2.5,  0.0] % y_i
    end % properties

    properties (Access = private)
        Particles_
        particleWeights_
        Pose
        previousOdomPose
        nSteps
        sampleMotionModel
        measurementModel
        measurementModelProbability
        getMeasurement
        getControl
    end % properties

    methods
        function obj = MonteCarloParticleFilter(varargin)
        % Constructor for the MCL object.
            setProperties(obj, nargin, varargin{:});
        end

        function handles = plot(obj)
        % PLOT Plot the filter estimates
            particles = obj.Particles_;
            particleHandle = scatter(particles(1,:), particles(2,:), ...
                                     'ko', 'markeredgealpha', 0.2);
            pose = obj.Pose;
            poseHandle = plot(pose(1), pose(2), 'o', ...
                              [pose(1), pose(1)+cos(pose(3))], ...
                              [pose(2), pose(2)+sin(pose(3))], ...
                              'color', 'k', 'markersize', 15, 'linewidth', 2);
            handles = vertcat(particleHandle, poseHandle);
        end
    end % methods

    methods (Access = protected)
        %% # TO DO
        % initialize THE PARTICLES and THEIR WEIGHTS according to the initial mean and the
        % initial covariance. Keep in mind the following variables:
        % obj.initialPose -> initial mean 
        % obj.initialCovariance -> initial covariance
        % obj.nParticle -> number of particles 
        % obj.Particles_ -> the particles
        % obj.particleWeights_ -> particles' weights
        function resetImpl(obj)
        % RESETIMPL Reset to initial state
            obj.Particles_ = (......)';
            obj.particleWeights_ = .....;
            
            obj.Pose = obj.weightedMean(obj.Particles_, obj.particleWeights_);
            obj.previousOdomPose = obj.initialPose;
            obj.nSteps = 0;
        end

        function setupImpl(obj, varargin)
        % SETUPIMPL Run one-time tasks to setup the object
            % Setup measurement model
            setupMeasurementModel(obj);
            % Setup motion model
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

            % Set up variables
            Particles = obj.Particles_;
            particleWeights = obj.particleWeights_;
            n = obj.nParticles;
            nEffectiveParticles = 1/sum(particleWeights.^2);
            particleCandidates = zeros(3,n);
            location = obj.landmarkLocation;
            visibleLandmarks = find(~isnan(measurement(1,:)));
            nVisibleLandmarks = length(visibleLandmarks);
            landmarkWeights = zeros(nVisibleLandmarks,n);

            %% # TO DO
            % MCL particle update
            for p = 1:n
                % Apply motion model to particles
                particleCandidates(:,p) = obj.sampleMotionModel(Particles(:,p), control);

                % Get weights for each measurement
                for idx = 1:nVisibleLandmarks
                    landmark = visibleLandmarks(idx);
                    landmarkWeights(idx, p) = obj.measurementProbability(...
                        particleCandidates(:,p), measurement(:,landmark), ...
                        location(:,landmark));
                end
            end

            if nVisibleLandmarks>0
                % Combine all landmarks
                landmarkWeights = prod(landmarkWeights,1);
                %% TO DO
                %landmarkWeights -> weights
                % particleWeights -> normalized weights
                
                % 1- normalize the weights
                % Set particle weight as the normalized landmark weights
                particleWeights = ;
                % 2- Calculate number of effective particles
                nEffectiveParticles = ;
            end  %if not seen, weights will remain the same as last cycle

            % Resample the particles every resemplingInterval
            obj.nSteps = obj.nSteps + 1;
            if nEffectiveParticles < obj.effectiveParticleThreshold*n || ...
                    mod(obj.nSteps, obj.resamplingInterval)==0
                % Resample particles
                Particles = datasample(particleCandidates, n, 2, ...
                                       'weights', particleWeights);
                particleWeights = ones(1,n)/n; % after resampling weights are equal

                if obj.useAugmentation
                    % Augment with random data
                    p = rand(1,n);
                    randomIndeces = p<=obj.augmentationProbability;
                    numRandom = sum(randomIndeces);
                    randomStates = obj.drawRandomStates(numRandom);
                    Particles(:,randomIndeces) = randomStates;
                end
            else
                Particles = particleCandidates;
            end % if not resampled, weights remain the same

            estimatedPose = obj.weightedMean(Particles, particleWeights);
            %% TO DO
            % Estimate the covariance 
            estimatedCovariance = ;
            obj.Particles_ = Particles;
            obj.particleWeights_ = particleWeights;
            obj.Pose = estimatedPose;
        end
    end % methods

    methods (Access = private)
        %% TO DO
        % Calculate the likelhood of the measurement given a state
        % (particle).
        % You should do that for two cases 1) with bearing only, 2) with
        % bearing and range.
        % obj.measurementModelProbability -> liklehood
        % Hint use normpdf
        function setupMeasurementModel(obj)
        % SETUPMEASUREMENTMODEL Setup measurement model
            switch obj.getNumInputs()
                case 2 % control angles
                    obj.measurementModel = @(pose, lm) ...
                        wrapToPi(atan2(lm(2) - pose(2), lm(1) - pose(1)) - pose(3));
                    
                    % 1-
                    obj.measurementModelProbability = @(measurement, expectedMeasurement)...
                        % To be completed;
                    
                    obj.getMeasurement = @(varargin)varargin{1}';
                case 3 % control + angles + ranges
                    obj.measurementModel = @(pose, lm) ...
                        [sqrt((lm(1) - pose(1))^2 + (lm(2) - pose(2))^2);
                         wrapToPi(atan2(lm(2) - pose(2), lm(1) - pose(1)) - pose(3))];
                    
                     % 2-
                     obj.measurementModelProbability = @(measurement, expectedMeasurement)...
                        % To be completed;;
                    obj.getMeasurement = @(varargin)[varargin{2:-1:1}]';
            end
        end

        function setupOdometryModel(obj)
        % SETUPODOMETRYMODELS Setup odometry motion model
            obj.getControl = @obj.getOdometryControl;
            obj.sampleMotionModel = @obj.sampleOdometryMotionModel;
        end

        function control = getOdometryControl(obj, odomPose)
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
            obj.getControl = @(control)control;
            obj.sampleMotionModel = @obj.sampleVelocityMotionModel;
        end

        function sample = sampleVelocityMotionModel(obj, pose, control)
        % SAMPLEMOTIONMODEL Draw a sample from the motion model
            noiseMod = sqrt(obj.robotNoise);
            a1 = noiseMod(1);
            a2 = noiseMod(2);
            a3 = noiseMod(3);
            a4 = noiseMod(4);
            a5 = noiseMod(5);
            a6 = noiseMod(6);
            v = control(1);
            w = control(2);
            dt = control(3);

            linearVelocitySample = v + (a1*abs(v) + a2*abs(w))*randn();
            angularVelocitySample = w + (a3*abs(v) + a4*abs(w))*randn();
            rotationSample = (a5*abs(v) + a6*abs(w))*randn();

            control = [linearVelocitySample; angularVelocitySample;
                       rotationSample; dt];

            sample = obj.velocityMotionModel(pose, control);
        end

        function sample = sampleOdometryMotionModel(obj, pose, control)
        % SAMPLEMOTIONMODEL Draw a sample from the motion model
            noiseMod = sqrt(obj.robotNoise);
            a1 = noiseMod(1);
            a2 = noiseMod(2);
            a3 = noiseMod(3);
            a4 = noiseMod(4);
            rotation1 = control(1);
            translation = control(2);
            rotation2 = control(3);

            rotation1Sample = ...
                wrapToPi(rotation1 - (a1*abs(rotation1) + a2*abs(translation))*randn());
            translationSample = ...
                translation - (a3*abs(translation) + a4*(abs(rotation1 + rotation2)))*randn();
            rotation2Sample = ...
                wrapToPi(rotation2 - (a1*abs(rotation2) + a2*abs(translation))*randn());

            control = [rotation1Sample; translationSample; rotation2Sample];

            sample = obj.odometryMotionModel(pose, control);
        end

        function probability = measurementProbability(obj, pose, measurement, landmark)
        % MEASUREMENTPROBABILITY Calculate probability for receiving the measurement
            expectedMeasurement = obj.measurementModel(pose, landmark);
            probability = obj.measurementModelProbability(measurement, expectedMeasurement);
        end

        function states = drawRandomStates(obj, num)
        % DRAWRANDOMSTATES Draw random states
            xlimit = obj.xlim;
            ylimit = obj.ylim;
            x = xlimit(1) + (xlimit(2) - xlimit(1))*rand(1,num);
            y = ylimit(1) + (ylimit(2) - ylimit(1))*rand(1,num);
            theta = -pi + 2*pi*rand(1,num);
            states = vertcat(x,y,theta);
        end
    end % methods

    methods (Static, Access = private)
        function pose = odometryMotionModel(pose, control)
        % ODOMETRYMOTIONMODEL Motion model for odometry control
            pose = [pose(1) + control(2)*cos(control(1) + pose(3));
                    pose(2) + control(2)*sin(control(1) + pose(3));
                    wrapToPi(control(1) + control(3) + pose(3))];
        end

        function pose = velocityMotionModel(pose, control)
        % VELOCITYMOTIONMODEL Motion model for velocity control
            x = pose(1);
            y = pose(2);
            th = pose(3);
            v = control(1);
            w = control(2);
            rot = control(3);
            dt = control(4);
            % Take into account the special case w=0
            if w==0
                dtv = dt*v;
                pose = [x + dtv*cos(th); y + dtv*sin(th); th + dt*rot];
            else
                r = v/w;
                th_dtw = th + dt*w;
                pose = [x + r*sin(th_dtw) - r*sin(th);
                        y - r*cos(th_dtw) + r*cos(th);
                        wrapToPi(th_dtw + dt*rot)];
            end
        end
        
        %% TO DO
        % Calculate the weighted mean for the particles.
        
        function mean = weightedMean(particles, weights)
        % WEIGHTEDMEAN Calculate the weighted mean of particles
            % Weighted Euclidean mean for the x-y values
            mean(1:2) = ;
            % Weighted mean for the orientation
            mean(3) = ;
        end

        function checkInputs(varargin)
        % CHECKINPUTS Validate inputs
            control = varargin{1};
            validateattributes(control, {'double'}, ...
                {'vector', 'numel', 3, 'real'}, 'step', 'control');

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
