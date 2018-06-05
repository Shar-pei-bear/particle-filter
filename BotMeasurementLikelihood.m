function  likelihood = BotMeasurementLikelihood(predictParticles, measurement)
   % The measurement contains all state variables
   predictMeasurement = predictParticles;

   % Calculate observed error between predicted and actual measurement
   measurementError = bsxfun(@minus, predictMeasurement(:,1:3), measurement'); % applies an element-by-element binary operation to arrays
   measurementErrorNorm = sqrt(sum(measurementError.^2, 2));

   % Normal-distributed noise of measurement
   % Assuming measurements on all three pose components have the same error distribution
   measurementNoise = eye(3);

   % Convert error norms into likelihood measure.
   % Evaluate the PDF of the multivariate normal distribution
   likelihood = 1/sqrt((2*pi).^3 * det(measurementNoise)) * exp(-0.5 * measurementErrorNorm);
end
