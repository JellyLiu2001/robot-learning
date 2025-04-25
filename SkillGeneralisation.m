classdef SkillGeneralisation
    % This SkillGeneralisation class implements the Dynamic Movement Primitives (DMPs).
    % It includes methods for preprocessing demonstration trajectories, performing nonlinear fitting,
    % and reconstructing trajectories. Some functions in this class are incomplete
    % and need to be implemented based on the knowledge from the Robot Learning and Teleoperation module.
    properties
        % Parameters for the DMPs
        numStates double            % Number of activation functions (i.e., number of RBF in the forcing term or K of a GMM )
        numVar int8                 % Number of variables dim([x,s1,s2]) (decay term and perturbing force)
        numVarPos int8              % Dimension of spatial variables dim([s1,s2])
        numData int16               % Number of time stamps of a trajectory
        numDemos int16              % Number of demonstrations
        beta double                 % Stiffness gain (β)
        alpha double                % Damping gain (α)
        dt double                   % Duration of time step (τ)
        L double                    % Feedback term
        xIn = []                    % State of the canonical system
        dataTraining = []           % Forcing term for training
        startPos double             % Start position of the trajectory
        endPos double               % End position of the trajectory
    end

    methods
        function obj = SkillGeneralisation(dim, states, beta, alpha, dt)
            % Construct an instance of this class
            obj.numStates = states;                 % Number of activation functions
            obj.numVar = dim;                       % Number of variables
            obj.numVarPos = obj.numVar-1;           % Dimension of spatial variables
            obj.beta = beta;                        % Stiffness gain
            obj.alpha = alpha;                      % Damping gain (with ideal underdamped damping ratio)
            obj.dt = dt;                            % Duration of time step
            obj.L = [eye(obj.numVarPos)*beta, eye(obj.numVarPos)*alpha]; % Feedback term
        end

        %% Generate time Stamps
        function obj = canonicalSystemInitialisation(obj,decayFactor,numData)
            obj.xIn(1) = 1;
            for t = 2:numData
                obj.xIn(t) = obj.xIn(t-1) - decayFactor * obj.xIn(t-1) * obj.dt;       % Update of decay term (dx/dt = -ax, τx'=-ax)
            end
            obj.numData = numData;
        end

        %% Compute imaginary force from trajectory
        function obj = trajectort2Forcing(obj, demos)
            obj.numDemos = length(demos);
            numTimeSteps = length(obj.xIn);
            obj.endPos = demos{1}.pos(:,end);
            obj.startPos = demos{1}.pos(:,1);
            for n = 1:obj.numDemos
                pos = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),numTimeSteps));     % Resampling Positions
                [traj(n).pos, traj(n).vel, traj(n).acc] = obj.generateHighOrderTerms(pos);
                oneDemoForceWithTimeStamps = [obj.xIn; (traj(n).acc - (repmat(obj.endPos,1,numTimeSteps)-traj(n).pos)*obj.beta + traj(n).vel*obj.alpha) ./ repmat(obj.xIn,obj.numVarPos,1)];
                obj.dataTraining = [obj.dataTraining, oneDemoForceWithTimeStamps];
            end
        end

        %% Compute velocity and acceleration from position trajectory
        function [pos, vel, acc] = generateHighOrderTerms(obj, pos)
            vel = gradient(pos) / obj.dt;             % Compute Velocity
            acc = gradient(vel) / obj.dt;             % Compute Acceleration
        end

        %% Ordinary DMP with locally weighed locally WLS
        function fOut = fittingWithLocallyWLS(obj)
            x = obj.dataTraining(1,:);           % phase variable (1 x T)
            f = obj.dataTraining(2:end,:)';      % forcing term (T x d)
            [T, d] = size(f);

            % Define RBF centers and widths
            c = linspace(min(x), max(x), obj.numStates);     % Centers of RBFs
            Sigma = repmat(2E-3, [1 1 obj.numStates]);        % Covariance (scalar for all)

            % Compute activation matrix H (nbStates x nbData)
            H = zeros(obj.numStates, obj.numData);
            for i = 1:obj.numStates
                H(i,:) = mvnpdf(obj.xIn', c(i), Sigma(:,:,i));  % Use canonical system phase for eval
            end
            H = H ./ (sum(H, 1) + realmin);  % Normalize across states

            % Expand H for all demonstrations
            H_all = repmat(H, 1, obj.numDemos);  % H: (K x T*numDemos)

            % Linear regression per RBF
            X = ones(obj.numDemos * obj.numData, 1);  % constant input for LWR
            Y = f;                                    % T x d
            WF = zeros(d, obj.numStates);            % Regression weights

            for i = 1:obj.numStates
                Psi = diag(H_all(i,:));              % Local activation matrix
                WF(:,i) = (X' * Psi * X \ X' * Psi * Y)';  % Solve LWR (d x 1)
            end

            % Reconstruct forcing function: currF = WF * H
            fOut = WF * H;    % Output: (d x T)
        end


        %% Optimised DMP with GMM and GMR
        function fOut = fittingWithGMR(obj)
            % -------------------Add your code here --------------------    👈👈👈
            % Prepare data explicitly: [phase, forcing terms]
            phase = obj.dataTraining(1,:)';                   % (T x 1)
            forcingTerms = obj.dataTraining(2:end,:)';        % (T x d), d为forcing维度

            gmmData = [phase, forcingTerms];                  % (T x (1+d))

            % Train GMM
            gmmodel = MixtureGaussians(gmmData, obj.numStates);
            gmmodel = gmmodel.gmmFit(gmmData, 200, 1e-6, false);

            % Define phase as query (input), forcing terms as response (output)
            mask = [1, zeros(1, size(forcingTerms,2))];       % [1 0 0 ...]
            gmmodel = gmmodel.defineQueryDim(mask);

            % GMR regression: query using canonical system (obj.xIn)
            queryGrid = obj.xIn';                             % 使用canonical phase作为查询点
            gmmodel = gmmodel.gaussianMixtureRegression(queryGrid);

            % Extract regressed forcing terms
            fOut = gmmodel.regressedTraj(:,2:end)';           % (d x T)

            % ----------------------------------------------------------    👈👈👈
        end

        %% Generalise imaginary force back to trajectory
        function trajOut = forcing2Trajectory(obj, forcingTraj)
            x = obj.startPos;
            xTarget = obj.endPos;
            dx = zeros(obj.numVarPos,1);
            for t = 1:obj.numData
                s = obj.xIn(t);  % phase variable
                ddx = obj.beta * (xTarget - x) - obj.alpha * dx + forcingTraj(:,t) * s;
                dx = dx + ddx * obj.dt;
                x = x + dx * obj.dt;
                rData(:,t) = x;
            end
            trajOut = rData;
        end
    end

    methods (Static)
        % ----- Add your functions here to help with your evaluation ------
        % -----------------------------------------------------------------
    end
end