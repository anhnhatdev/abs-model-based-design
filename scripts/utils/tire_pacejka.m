function mu = tire_pacejka(lambda, road_type)
% TIRE_PACEJKA Calculates friction coefficient mu based on slip ratio lambda
% 
% Inputs:
%   lambda    - Slip ratio [0.0 to 1.0]
%   road_type - Road surface selector:
%               1 = Dry Asphalt (mu_peak ~ 0.90)
%               2 = Wet Asphalt (mu_peak ~ 0.50)
%               3 = Gravel / Ice (mu_peak ~ 0.30)
%
% Output:
%   mu        - Friction coefficient []

% Clamp slip ratio input to valid physical range [0, 1]
lambda = max(0.0, min(1.0, lambda));

% Select parameters based on road surface
switch road_type
    case 1 % Dry Asphalt
        B = 10.0; C = 1.90; D = 0.90; E = 0.97;
    case 2 % Wet Asphalt
        B = 8.0;  C = 1.70; D = 0.50; E = 0.80;
    case 3 % Gravel / Ice
        B = 6.0;  C = 1.50; D = 0.30; E = 0.60;
    otherwise % Default to Dry Asphalt
        B = 10.0; C = 1.90; D = 0.90; E = 0.97;
end

% Pacejka Magic Formula calculation
phi = B * lambda;
mu = D * sin(C * atan(phi - E * (phi - atan(phi))));

% Ensure mu is non-negative
mu = max(0.0, mu);

end
