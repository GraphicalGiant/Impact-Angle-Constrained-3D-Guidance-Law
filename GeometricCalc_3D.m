


V = [40, 50, 100]; % Velocity vector
VM = norm(V);

% rt = [2500,1000,800];
rt = [2600,1040,520];  %3D

% rt = [3000,1200,600];   
missile_position = [0, 0, 0];           % Missile initial position
planePoint = [0, 0, 0];         % A point on the plane
alpha_f = alpha_f_values(i);  %3D
x = [missile_position(1),rt(1)];
y = [missile_position(2),rt(2)];
z = [missile_position(3),rt(3)];
  
lineDirection = (rt-missile_position);   % Line direction vector
radius = 400;                     % Radius of the circle

Vf = findVector(lineDirection,VM ,alpha_f);    %3D
normal1 = cross(lineDirection,V)/norm(cross(lineDirection,V));
normal2 = cross(lineDirection,Vf)/norm(cross(lineDirection,Vf)); 

% Find the center of the circle
C1 = findCircleCenterTangential(V, missile_position, missile_position, lineDirection, radius,2);

C2 = findSecondCircleCenter(C1, radius, normal1, radius, missile_position, lineDirection,2);

C4 = findCircleCenterTangential(Vf, rt, rt ,lineDirection, radius,2);  %----->

C3 = findSecondCircleCenter(C4, radius , -normal2, radius, rt, lineDirection,1);  %----->

% Call the function
circle1 = plot3DCircle(C1, radius, normal1);

circle2 = plot3DCircle(C2,radius, normal1);

circle4 = plot3DCircle(C4, radius, normal2);  %---->

circle3 = plot3DCircle(C3, radius,normal2);   %---->

scale = 200;


[P1,P2] = findPointsOfContact(C1,radius,C2,radius, lineDirection,normal1,1);
[P4,P3] = findPointsOfContact(C4, radius, C3, radius, lineDirection, -normal2,1); %----->




function center = findCircleCenterTangential(velocity, position, planePoint, lineDirection, radius, flag)
    % Inputs:
    % velocity: Initial velocity vector [vx, vy, vz]
    % position: Missile's initial position [x_m, y_m, z_m]
    % planePoint: A point on the plane [x_p, y_p, z_p]
    % lineDirection: Direction vector of the line [d_x, d_y, d_z]
    % radius: Radius of the circle
    % flag: Determines which solution to use (1 for second, 0 for first)

    % Normalize the velocity vector
    velocityUnit = velocity / norm(velocity);

    % Compute the normal vector of the plane
    normal = cross(velocity, lineDirection);
    normal = normal / norm(normal); % Normalize the normal vector

    % Symbolic variables for the center
    syms x y z

    % Define the center of the circle as [x, y, z]
    C = [x, y, z];

    % Constraint 1: Plane equation -> (C - planePoint) . normal = 0
    planeEq = dot(C - planePoint, normal) == 0;

    % Constraint 2: Distance from position to center equals radius -> ||C - position|| = radius
    radiusConstraint = norm(C - position) == radius;

    % Constraint 3: Tangency condition -> (velocityUnit . (C - position)) = 0
    tangencyConstraint = dot(velocityUnit, C - position) == 0;

    % Solve the system of equations
    sol = solve([planeEq, radiusConstraint, tangencyConstraint], [x, y, z], 'Real', true);

    % Extract solutions for the center
    centers = double([sol.x, sol.y, sol.z]);

    if flag == 1
        % Use the second solution
        center = centers(2, :);
    else
        % Use the first solution
        center = centers(1, :);
    end
end



function circle3D =  plot3DCircle(center, radius, normal)
    % Inputs:
    % center: [x, y, z] - Center of the circle in 3D
    % radius: r - Radius of the circle
    % normal: [nx, ny, nz] - Normal vector of the plane of the circle

    % Define the number of points to plot the circle
    theta = linspace(0, 2*pi, 100); % Parametric angle
    circle2D = radius * [cos(theta); sin(theta); zeros(size(theta))]; % Circle in 2D

    % Normalize the normal vector
    normal = normal / norm(normal);

    % Find two vectors orthogonal to the normal
    if all(normal == [0, 0, 1])
        % Special case: Normal is aligned with Z-axis
        u = [1, 0, 0];
        v = [0, 1, 0];
    else
        % General case
        u = cross(normal, [0, 0, 1]); % Arbitrary vector orthogonal to normal
        u = u / norm(u); % Normalize
        v = cross(normal, u); % Second orthogonal vector
    end

    % Transform the 2D circle to 3D
    rotationMatrix = [u; v; normal]'; % Matrix to rotate the circle into 3D
    circle3D = rotationMatrix * circle2D;

    % Translate the circle to the center
    circle3D = circle3D + center';
    % scale = 40;
end


function C2 = findSecondCircleCenter(center, r1, planeNormal, r2, linePoint, lineDirection, flag)
   
    % planeNormal: Normal vector of the plane [nx, ny, nz]
    % linePoint: A point on the line [x_p, y_p, z_p]
    % lineDirection: Direction vector of the line [d_x, d_y, d_z]

    syms x y z t
    % Equation 1: Distance between centers equals r1 + r2
    eq1 = sqrt((x - center(1))^2 + (y - center(2))^2 + (z - center(3))^2) == (r1 + r2);

    % Equation 2: Distance from the center to the line equals r2
    linePointToCenter = [x, y, z] - linePoint;
    crossProduct = cross(lineDirection,linePointToCenter);
    % crossProductSq = sum(crossProduct.^2);
    % lineDirectionSq = sum(lineDirection.^2);
    distanceToLine = dot(planeNormal,crossProduct) / norm(lineDirection);
    % distanceToLine = sqrt(crossProductSq/lineDirectionSq);
    
    eq2 = distanceToLine == r2;
    % eq2 = (x - linePoint(1))^2 + (y - linePoint(2))^2 + (z - linePoint(3))^2 - t^2 * norm(lineDirection)^2 == r2^2;


    % Equation 3: Point lies on the plane
    eq3 = planeNormal(1)*(x - center(1)) + planeNormal(2)*(y - center(2)) + planeNormal(3)*(z - center(3)) == 0;

    % Solve the system of equations
    sol = vpasolve([eq1, eq2, eq3], [x, y, z]);

  
    % Extract the first solution (C2_1)
    C2_1 = double([sol.x, sol.y, sol.z]) 


    % computing the circle on the otherside
    % Compute the vector from the first solution to the line
    P_lineDirection = cross(planeNormal,lineDirection);
    CToC2_1 = C2_1 - center;
    projection = (dot(CToC2_1, P_lineDirection) / norm(P_lineDirection)^2 )* P_lineDirection;
    parallelVector = CToC2_1 - projection;
    C2_2 = C2_1 - 2 * parallelVector;
   
    % Return the solution based on the flag (which one to return)
    if flag == 1
        C2 = C2_1;
    else
        C2 = C2_2;
    end
end

function [P_circle, P_line] = findPointsOfContact(C1, r1, C2, r2,lineDirection, normal,flag)
    % Inputs:
    % C1: Center of the first circle [x1, y1, z1]
    % r1: Radius of the first circle
    % C2: Center of the second circle [x2, y2, z2]
    % r2: Radius of the second circle
    % linePoint: A point on the line [x_p, y_p, z_p]
    % lineDirection: Direction vector of the line [d_x, d_y, d_z]

    % 1. Point of Contact Between the Two Circles
    D = C2 - C1; % Direction vector from C1 to C2
    D_unit = D / norm(D); % Normalize the direction vector
    P_circle = C1 + r1 * D_unit; % Point of contact between the circles

    % 2. Point of Contact Between the Second Circle and the Line
  
        P_lineDirection = cross(normal,lineDirection);

        P_line = C2-flag*(r2*P_lineDirection/norm(P_lineDirection));

   
end

function V = findVector(R0, V_mag, alpha)
% FINDVECTORNONORMAL Compute a vector V in the same plane as L at a given angle alpha.
% Inputs:
%   L      - Known vector in 3D (1x3 or 3x1 array)
%   V_mag  - Magnitude of the desired vector V (scalar)
%   alpha  - Angle (in degrees) between V and L (scalar)
%
% Output:
%   V      - The resultant vector V (1x3 array)

    % Normalize vector L
    R0_unit = R0 / norm(R0);

    % Generate an arbitrary vector that is not parallel to L
    arbitrary = [1, 0, 0];
    if abs(dot(R0_unit, arbitrary)) == 1 % Check if parallel
        arbitrary = [0, 1, 0];
    end

    % Find a perpendicular vector to L
    L_perp = cross(R0_unit, arbitrary);
    L_perp_unit = L_perp / norm(L_perp); % Normalize

    % Calculate components of V
    V_parallel = V_mag * cosd(alpha) * R0_unit; % Parallel component
    V_perpendicular = V_mag * sind(alpha) * L_perp_unit; % Perpendicular component

    % Compute the resultant vector V
    V = V_parallel + V_perpendicular;
end
