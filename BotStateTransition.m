function predictParticles = BotStateTransition(prevParticles, dT, u)
   thetas = prevParticles(:,3);
   w = u(2);
   v = u(1);
   n = length(prevParticles);
   
   % Generate velocity samples
   sd1 = 0.3;  % sd1 represents the uncertainty in the linear velocity
   sd2 = 1.5;  % sd2 represents the uncertainty in the angular velocity
   sd3 = 0.02; % sd3 is an additional perturbation on the orientation
   vh = v + (sd1)^2*randn(n,1);
   wh = w + (sd2)^2*randn(n,1);
   gamma = (sd3)^2*randn(n,1);
   
   % Add a small number to prevent div/0 error
   wh(abs(wh)<1e-19) = 1e-19;
   
   % State Transition
   predictParticles(:,1) = prevParticles(:,1) - (vh./wh).*sin(thetas) + (vh./wh).*sin(thetas + wh*dT);
   predictParticles(:,2) = prevParticles(:,2) + (vh./wh).*cos(thetas) - (vh./wh).*cos(thetas + wh*dT);
   predictParticles(:,3) = prevParticles(:,3) + wh*dT + gamma*dT;
   predictParticles(:,4) = (- (vh./wh).*sin(thetas) + (vh./wh).*sin(thetas + wh*dT))/dT;
   predictParticles(:,5) = ( (vh./wh).*cos(thetas) - (vh./wh).*cos(thetas + wh*dT))/dT;
   predictParticles(:,6) = wh + gamma;
end

