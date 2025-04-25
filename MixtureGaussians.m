classdef MixtureGaussians
    % The MixtureGaussians class encapsulates the following functionalities:
    % Gaussian Mixture Model (GMM): Represents a mixture of Gaussian distributions.
    % Fits the model to data using the EM algorithm. Computes the log-likelihood
    % of the data given the model. Visualizes the GMM and data.
    % Gaussian Mixture Regression (GMR): Performs regression using the GMM.
    % Predicts output dimensions given input dimensions.
    % Visualizes regression results with confidence intervals.

    properties
        % GMM parameters
        k int8;
        pi double;
        mus (:,:) double;
        sigmas (:,:,:) double;
        logLikelihood = [];
        % GMR parameters
        d int8;
        inDim int8;
        outDim int8;
        regressedTraj = [];
        confidence = [];
    end

    methods
        function obj = MixtureGaussians(data, K)
            % Construct an instance of this class
            % Create a GMM parameter set with initialisation
            if nargin < 2
                K = 5;
            end
            [N,D] = size(data);

            % initialise GMM parameters
            obj.k = K;
            obj.pi = ones(1, K) / K;            % Initialize mixing coefficients (weights) equally
            randIdx = randperm(N, K);           % Randomly choose K data points as initial means (each column is a mean vector)
            obj.mus = data(randIdx, :)';        % d x K matrix;
            obj.sigmas = repmat(cov(data), [1, 1, K]); % Initialise covariances as the sample covariance
            obj.logLikelihood = -inf;           % Initialise log likelihood for convergence checking

            % initialise GMR parameters
            obj.d = D;
            obj.inDim = 1;
            obj.outDim = [];
        end

        %% GMM functions
        function obj = emStep(obj,data)
            % Update GMM parameters with one EM iteration
            [obj.pi, obj.mus, obj.sigmas] = MixtureGaussians.oneEmStep(data, obj.pi, obj.mus, obj.sigmas);
        end

        function obj = gmmFit(obj, data, maxIter, cvgTol, plotFig)
            % Full EM loop
            % maxIter: Set maximum number of iterations
            % CvgTol: Convergence tolerance for the log-likelihood
            % plotFig = true: Plot the GMM with data
            if nargin < 2
                maxIter=200;
                cvgTol=1e-6;
                plotFig=false;
            end

            if plotFig
                figure;
            end
            %% EM loop
            for iter = 1:maxIter
                obj = emStep(obj, data);

                % Compute the log likelihood for the current parameters.
                LogLOld = obj.logLikelihood;
                LogLNew = obj.computeLoglikelihood(data);
                % Check for convergence: if the change in log-likelihood is below tolerance, stop iterating.
                if abs(LogLNew-LogLOld) < cvgTol
                    fprintf('Convergence reached at iteration %d.\n', iter);
                    break;
                end
                obj.logLikelihood(iter) = LogLNew;
                if plotFig
                    % Plot the GMM after each iteration.
                    obj.plotGmmAndData(data)
                    title('GMM fitting using the EM Algorithm');
                    grid on;
                    axis equal;
                    pause(0.1);
                    fprintf('Iteration %d: Log-Likelihood = %.6f\n', iter, LogL_new);
                end
            end
        end

        function plotGmmAndData(obj, data)
            if nargin < 1
                plotWithData = false;
            else
                plotWithData = true;
            end

            data_dim = size(data,2);
            if data_dim == 1
                disp('Unable to plot the 1-d data!')
            elseif data_dim == 2
                if plotWithData
                    scatter(data(:,1), data(:,2), 10, '.b'); hold on;
                end
                MixtureGaussians.plotGaussians(obj.mus, obj.sigmas)
                hold off
            else
                if plotWithData
                    scatter3(data(:,1), data(:,2), data(:,3), 10, '.b'); hold on;
                end
                MixtureGaussians.plotGaussians(obj.mus, obj.sigmas)
                hold off
            end
        end

        function logL = computeLoglikelihood(obj, data)
            % Compute the log likelihood with dataset 'data'.
            [N,~] = size(data);
            temp = zeros(N,obj.k);
            for j = 1:obj.k
                temp(:,j) = obj.pi(j) * mvnpdf(data, obj.mus(:, j)', obj.sigmas(:, :, j));
            end
            logL = sum(log(sum(temp,2)),'all');
        end

        %% GMR functions
        function obj = defineQueryDim(obj, mask)
            % Use the mask vector to define the Query dimensions the rest of
            % the dimensions will be the Response dimensions
            dims = 1:1:obj.d;
            obj.inDim = dims(mask==1);
            obj.outDim = dims(mask==0);
        end

        function obj = gaussianMixtureRegression(obj, querys)
            T = size(querys,1);
            response = zeros(T,length(obj.outDim));
            confidenceInterval = zeros(length(obj.outDim),length(obj.outDim),T);
            for t = 1:T
                [response(t,:), confidenceInterval(:,:,t)] = MixtureGaussians.oneGmr(querys(t,:), obj.inDim, obj.outDim, obj.pi, obj.mus, obj.sigmas);
            end
            obj.regressedTraj = [querys,response];
            obj.confidence = confidenceInterval;
        end

        function plotGmrResults(obj, data)
            querys = obj.regressedTraj(:,obj.inDim);
            responses = obj.regressedTraj(:,obj.outDim);
            dims = size(responses,2);
            for m=1:dims
                confidences = obj.confidence;
                %% GMM result
                subplot(dims,2,m); hold on;
                % Plot training data
                scatter(data(:,1), data(:,m+1), '.', 'MarkerEdgeColor', [.8 .8 .8] );
                % Draw GMM ellipses
                MixtureGaussians.plotGaussians(obj.mus([1,1+m],:),obj.sigmas([1,1+m],[1,1+m],:));
                set(gca,'xtick',[],'ytick',[]);
                xlabel('Query dim');
                ylabel(['Response dim ' num2str(m)]);
                %% GMR result
                subplot(dims,2,m+2); hold on;
                % Plot the regressed trajectory by the GMR
                scatter(data(:,1), data(:,m+1), '.', 'MarkerEdgeColor', [.9 .9 .9] );
                plot(querys(:,1), responses(:,m), '-','lineWidth',2,'color',[0 1 0]);
                % Plot the confidence interval of the GMR
                patch([querys(:,1)', querys(end:-1:1,1)'], ...
                    [responses(:,m)'+squeeze(confidences(m,m,:).^.8)', responses(end:-1:1,m)'-squeeze(confidences(m,m,end:-1:1).^.8)'], ...
                    [.8 .3 .3],'edgecolor','none','facealpha',.4);
                % Plot control
                set(gca,'xtick',[],'ytick',[]);
                xlabel('Query dim');
                ylabel(['Response dim ' num2str(m)]);
                axis([querys(1,:), querys(end,:), 1.2*min(responses(:,m)), 1.2*max(responses(:,m))]);
            end
        end
    end % of non-static methods


    methods (Static)
        function plotGaussians(Mu,Sigma)
            [~, data_dim, num_gaussian] = size(Sigma);
            if data_dim == 1
                disp('Increase the dimension of data to see the plot of Gaussians')
            elseif data_dim == 2
                for ng = 1:num_gaussian
                    mu2ds = Mu(:,ng);             % Mean (2x1 vector)
                    Sigma2ds = Sigma(:,:,ng);     % Covariance matrix (2x2)
                    MixtureGaussians.plot2DGaussian(mu2ds,Sigma2ds);
                end
            else
                for ng = 1:num_gaussian
                    mu3ds = Mu(1:3,ng);           % Mean (3x1 vector)
                    Sigma3ds = Sigma(1:3,1:3,ng); % Covariance matrix (3x3)
                    MixtureGaussians.plot3DGaussian(mu3ds,Sigma3ds);
                end
                % Rendering
                l1 = light;
                l1.Position = [160 400 80];
                l1.Style = 'local';
                l1.Color = [0.8 0.8 0.3];
            end
        end

        function plot3DGaussian(mu3d,sigma3d)
            % Compute the eigenvalues and eigenvectors of the covariance matrix
            [V, D] = eig(sigma3d);
            p = 0.7;  % Determine the scaling factor for the desired confidence interval (95%)
            c = sqrt(chi2inv(p, 3));   % chi2inv returns the chi-square inverse cumulative density value
            n = 50;                    % Mesh grid resolution
            [x, y, z] = ellipsoid(0, 0, 0, 1, 1, 1, n);
            % Reshape the sphere data into 2D arrays for transformation
            points = [x(:)'; y(:)'; z(:)'];
            % Transform the unit sphere into an ellipsoid corresponding to the Gaussian
            % Multiply by sqrt(D) scales each axis by the standard deviation
            % V rotates the sphere to align with the covariance
            transformed_points = V * sqrt(D) * c * points;
            % Translate the ellipsoid by adding the mean vector
            transformed_points = bsxfun(@plus, transformed_points, mu3d);
            % Reshape the transformed coordinates back into meshgrid format
            x_ellipsoid = reshape(transformed_points(1, :), size(x));
            y_ellipsoid = reshape(transformed_points(2, :), size(y));
            z_ellipsoid = reshape(transformed_points(3, :), size(z));
            % Plot the ellipsoid with semi-transparency
            surf(x_ellipsoid, y_ellipsoid, z_ellipsoid, ...
                'FaceAlpha', 0.3, ...    % Set transparency (0 = transparent, 1 = opaque)
                'EdgeColor', 'none', ...
                'FaceColor', 'red', 'FaceLighting', 'gouraud');    % Remove mesh lines for a smoother appearance
            hold on;
            % Optionally, plot the mean as a red star marker
            plot3(mu3d(1), mu3d(2), mu3d(3), 'w+', 'MarkerSize', 10,'LineWidth',2);
        end

        function plot2DGaussian(mu2d,sigma2d)
            p = 0.7;  % Determine the scaling factor for the desired confidence interval (95%)
            c = sqrt(chi2inv(p, 2));  % chi2inv returns the chi-square inverse cumulative value for 2 degrees of freedom
            % Compute the eigenvalues and eigenvectors of the covariance matrix
            [V, D] = eig(sigma2d);
            % Generate points on a unit circle
            theta = linspace(0, 2*pi, 100);  % 100 points around the circle
            unitCircle = [cos(theta); sin(theta)];  % 2 x 100 matrix
            % Transform the unit circle into an ellipse corresponding to the Gaussian
            % The transformation scales by the square root of eigenvalues and rotates using V,
            % then scales by the chi-square factor and translates by the mean.
            ellipse = bsxfun(@plus, c * V * sqrt(D) * unitCircle, mu2d);
            % Plot the ellipse with semi-transparency
            fill(ellipse(1, :), ellipse(2, :), 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
            hold on;
            % Plot the mean as a red marker
            plot(mu2d(1), mu2d(2), 'w+', 'MarkerSize', 10, 'LineWidth',2);
        end

        function [pi_k_new,mu_k_new,Sigma_k_new] = oneEmStep(X,pi_k,mu_k,Sigma_k)
            arguments (Input)
                X (:,:) double
                pi_k (1,:) double     % must be a row vector of doubles
                mu_k (:,:) double {MixtureGaussians.mustBeValidMeans(X, mu_k)}
                Sigma_k (:,:,:) double {MixtureGaussians.mustBeValidSigmas(X,Sigma_k)}
            end

            arguments (Output)
                pi_k_new (1,:) double   % must be a row vector of doubles
                mu_k_new double
                Sigma_k_new double
            end

            [N,d] = size(X);
            K = length(pi_k);
            gamma = zeros(N, K);                 % Initialise posterior p(z|x)
            mu_k_new = zeros(size(mu_k));        % Preallocate Mus
            Sigma_k_new = zeros(size(Sigma_k));  % Preallocate Sigmas

            % -------------------Add your code here --------------------    👈👈👈
            %% E-step
            for k = 1:K
                gamma(:,k) = pi_k(k) * mvnpdf(X, mu_k(:,k)', Sigma_k(:,:,k));
            end
            gamma = gamma ./ sum(gamma, 2);  % Normalize to sum to 1 across clusters
            Nk = sum(gamma, 1);              % Effective number of points per cluster
            pi_k_new = Nk / N;              % New mixing coefficients


            %% M-step
            for k = 1:K
                % Update mean
                mu_k_new(:,k) = (X' * gamma(:,k)) / Nk(k);

                % Update covariance
                X_centered = X - mu_k_new(:,k)';
                Sigma_k_new(:,:,k) = (X_centered' .* gamma(:,k)') * X_centered / Nk(k);

                % Regularization to avoid singular matrix
                Sigma_k_new(:,:,k) = Sigma_k_new(:,:,k) + 1e-6 * eye(d);
            end
            % ----------------------------------------------------------    👈👈👈
        end

        function mustBeValidMeans(X, mu)
            % Check the format of GMM means
            if size(X, 2) ~= size(mu, 1)
                error("Mus must have the size (%d, K).", size(X, 2));
            end
        end

        function mustBeValidSigmas(X, sigmas)
            % Check the format of GMM covariance
            d = size(X, 2);
            if (d ~= size(sigmas, 1)) | (d ~= size(sigmas, 2))
                error("Sigmas must have the size (%d, %d, K).",d,d);
            end
        end

        function [expData, expSigma] = oneGmr(query, in, out, prior, mu, sigma)
            % 'in' represent the vector of the time and 'out' represent the desired output of trajectory
            % -------------------Add your code here --------------------    👈👈👈
            nbStates = length(prior);
            queryDim = length(in);
            respDim = length(out);

            Pxi = zeros(nbStates,1);
            beta = zeros(nbStates,1);
            expData = zeros(respDim,1);
            expSigma = zeros(respDim, respDim);
            expData_k = zeros(respDim, nbStates); % 每个状态单独预测的均值

            for k = 1:nbStates
                % 计算每个高斯成分的条件概率 p(query|in)
                Pxi(k) = prior(k) * mvnpdf(query, mu(in,k)', sigma(in,in,k));
            end

            % 正规化
            beta = Pxi / sum(Pxi + realmin); % 避免除0

            % 计算条件期望（条件均值）
            for k = 1:nbStates
                expData_k(:,k) = mu(out,k) + sigma(out,in,k) / sigma(in,in,k) * (query' - mu(in,k));
                expData = expData + beta(k) * expData_k(:,k);
            end

            % 计算条件方差（包括均值的不确定性）
            for k = 1:nbStates
                diff_mu = expData_k(:,k) - expData; % 与整体均值的偏差
                expSigma = expSigma + beta(k) * (sigma(out,out,k) - sigma(out,in,k)/sigma(in,in,k)*sigma(in,out,k) + diff_mu*diff_mu');
            end
            % ----------------------------------------------------------    👈👈👈
        end


    end % of static methods
end