function plotGaussians(Mu,Sigma)

[~, data_dim, num_gaussian] = size(Sigma);
p = 0.7;  % Determine the scaling factor for the desired confidence interval (95%)

if data_dim == 1
    disp('Increase the dimension of data to see the plot of Gaussians')
elseif data_dim == 2
    for ng = 1:num_gaussian
        mu2ds = Mu(:,ng);             % Mean (2x1 vector)
        Sigma2ds = Sigma(:,:,ng);         % Covariance matrix (2x2)
        plot2DGaussian(mu2ds,Sigma2ds);
    end
else
    for ng = 1:num_gaussian
        mu3ds = Mu(:,ng);          % Mean (3x1 vector)
        Sigma3ds = Sigma(:,:,ng);      % Covariance matrix (3x3)
        plot3DGaussian(mu3ds,Sigma3ds);
    end
    % Rendering
    l1 = light;
    l1.Position = [160 400 80];
    l1.Style = 'local';
    l1.Color = [0.8 0.8 0.3];
end


%% Functions
    function plot3DGaussian(mu3d,sigma3d)
        % Compute the eigenvalues and eigenvectors of the covariance matrix
        [V, D] = eig(sigma3d);
        c = sqrt(chi2inv(p, 3));   % chi2inv returns the chi-square inverse cumulative density value
        % Generate a unit sphere using the 'ellipsoid' function
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
        % Adjust the axes for equal scaling and enable grid
        % axis equal;
        % grid on;
        % hold off;
    end

    function plot2DGaussian(mu2d,sigma2d)
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
        % Add labels and title
        % xlabel('X');
        % ylabel('Y');
        % title('2D Gaussian Distribution: 95% Confidence Ellipse');
        % grid on;
        % axis equal;  % Ensure equal scaling for x and y axes
    end

end

