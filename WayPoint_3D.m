
alpha_f_values = [60, 120];
fig1 = figure(1);
% hold on;
grid on;
box on;

% Declaring placeholders
legend_entries = [];
LeadAngle_profiles0 = {};
time_values0 = {};
time_values1 = {};
LOSAngle_profiles = {};
flightAngle_profiles = {};
headingAngle_profiles = {};
acceleration_profiles = {};


% to generate results for multiple alpha_f values
for i = 1:numel(alpha_f_values)

    GeometricCalc_3D

    transition_points = [P1; P2; P3; P4; rt];

    % Time and Initialization
    dt = 0.00001;
    end_time = 50; % Allow sufficient time for full trajectory
    time = 0:dt:end_time;
    missile_path = zeros(length(time), 3);
    transition_index = 1;
    Lead_angle = [];
    LOS_angle = [];
    time_record = [];
    time_record1 = [];
    flight_path_angle = [];
    acceleration_profile = [];
    heading_angle = [];
    lead_angle_index = 0; % Initialize outside the loop

    %  PN Guidance Loop
    for t = 1:length(time)
        if transition_index > size(transition_points, 1)
            break; % Exit if all transition points are reached
        end
        if norm(missile_position -rt)< 0.5
            break;
        end
        % Proportional Navigation Guidance
        los_vector = transition_points(transition_index, :) - missile_position;
        los_rate = cross(V, los_vector) / norm(los_vector)^2; % LOS angular rate

        % if lead_angle_index > 0 && Lead_angle(lead_angle_index) <= 60
            accel_command = 2* cross(los_rate, V); % PN guidance (n=2)
        % else
            % accel_command = 4*cross(los_rate, V); % PN guidance (n=1)
        % end
    
        % Record acceleration magnitude
        % acceleration_profile(end+1) = norm(accel_command);
        % time_record1(end+1) = time(t);
       
         % Calculate Lead Angle, Flight path angle & LOS Angle for 3D
         if mod(t,10) == 0
            lead_angle_index = lead_angle_index + 1;
            Lead_angle(end+1) = acosd(dot(V, rt-missile_position) / (norm(V) * norm(rt-missile_position)));
            % LOSI = rt - missile_position;
            % LOS_angle(end+1) = atan2d(LOSI(3),sqrt(LOSI(1)^2+LOSI(2)^2));
            % flight_path_angle(end+1) = atan2d(V(3),sqrt(V(1)^2+V(2)^2));
            % heading_angle(end+1) = atan2d(V(2),V(1));
            time_record(end+1) = time(t);
         end

        % Update Velocity and Position
        V = V + accel_command * dt; % Adjust velocity
        missile_position = missile_position + V * dt; % Update position
        missile_path(t, :) = missile_position;

        % Transition to Next Point
        if norm(missile_position - transition_points(transition_index, :)) < 0.1
            transition_index = transition_index + 1;
        end
    end

    % Storing the values and plotting the results
    missile_path = missile_path(any(missile_path, 2), :);
 
    % Plot Trajectory
    % figure
    hold on;
    h = plot3(missile_path(:,1), missile_path(:,2), missile_path(:,3), ...
        'DisplayName',sprintf("\\alpha_f = %.0f^\\circ",alpha_f),LineWidth=2);
    legend_entries = [legend_entries,h];

    % Storing Lead angle and LOS angle data
    LeadAngle_profiles0{end+1} = Lead_angle;
    % flightAngle_profiles{end+1} = flight_path_angle;
    % LOSAngle_profiles{end+1} = LOS_angle;
    % acceleration_profiles{end+1} = acceleration_profile;
%     headingAngle_profiles{end+1} = heading_angle;
    time_values0{end+1} = time_record;
    time_values1{end+1} = time_record1;
    

    % Plot Circles in 3D (projected on the x-y plane)
    % %     figure
    plot3(circle1(1, :), circle1(2, :), circle1(3, :), 'k-.', 'LineWidth', 0.1);
    %     hold on;
    plot3(circle2(1, :), circle2(2, :), circle2(3, :), 'k-.', 'LineWidth', 0.1);

    plot3(circle4(1, :), circle4(2, :), circle4(3, :), 'k-.', 'LineWidth', 0.1);
    plot3(circle3(1, :), circle3(2, :), circle3(3, :), 'k-.', 'LineWidth', 0.1);


    % Plot center and normal vector
    % plot3(C1(1), C1(2), C1(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    % quiver3(C1(1), C1(2), C1(3), scale*normal1(1), scale*normal1(2), scale*normal1(3), ...
    %          'k', 'LineWidth', 0.2, 'MaxHeadSize',0.5);

    % plot3(C2(1), C2(2), C2(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    % quiver3(C2(1), C2(2), C2(3), scale*normal1(1), scale*normal1(2), scale*normal1(3), ...
    %         'r', 'LineWidth', 0.2, 'MaxHeadSize',0.5);

    % plot3(C4(1), C4(2), C4(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    % quiver3(C4(1), C4(2), C4(3), scale*normal2(1), scale*normal2(2), scale*normal2(3), ...
    %         'r', 'LineWidth', 0.2, 'MaxHeadSize',0.5);

    % plot3(C3(1), C3(2), C3(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    % quiver3(C3(1), C3(2), C3(3), scale*normal2(1), scale*normal2(2), scale*normal2(3), ...
    %         'r', 'LineWidth', 0.2, 'MaxHeadSize',0.5);
    quiver3(0, 0, 0, scale*3, scale*0, scale*0, 'k', 'LineWidth', 0.2, 'MaxHeadSize',0.3);
    quiver3(0, 0, 0, scale*0, scale*3, scale*0, 'k', 'LineWidth', 0.2, 'MaxHeadSize',0.3);
    quiver3(0, 0, 0, scale*0, scale*0, scale*3, 'k', 'LineWidth', 0.2, 'MaxHeadSize',0.3);

    quiver3(2600, 1040, 520, scale*Vf(1)/VM, scale*Vf(2)/VM, scale*Vf(3)/VM,'k','LineWidth', 1, 'MaxHeadSize',1);

    % quiver3(0, 0, 0, scale*(40)/VM, scale*50/VM, scale*100/VM, 'r', 'LineWidth', 2, 'MaxHeadSize',0.5);


    
    % Plot Transition Points
    plot3(transition_points(:,1), transition_points(:,2), transition_points(:,3), ...
        'bx', 'MarkerSize', 5, 'LineWidth', 1.5);
   
    % Line with markers at the endpoints % Line with markers at the endpoints
     plot3(x, y, z, 'k-.', 'LineWidth', 0.3);

   
end
   % Adjust plot appearance
axis equal;
grid on;     
legend(legend_entries); 
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
hold off;


fig2 = figure(2);
hold on;
for i = 1:numel(alpha_f_values)
    plot(time_values0{i}, LeadAngle_profiles0{i}, 'LineWidth', 1.5, ...
        'DisplayName', sprintf("\\alpha_f = %.0f^\\circ", alpha_f_values(i)));
end
xlabel('Time (s)');
ylabel("\sigma (deg)");
legend('show');
grid on;
box on;
hold off;

% fig3 = figure(3);
% hold on;
% for i = 1:numel(alpha_f_values)
%     plot(time_values{i}, LOSAngle_profiles{i}, 'LineWidth', 1.5, ...
%         'DisplayName', sprintf("\\alpha_f = %.0f^\\circ", alpha_f_values(i)));
% end
% xlabel('Time (s)');
% ylabel("\theta (deg)");
% legend('show');
% grid on;
% box on;
% hold off;

% fig4 = figure(4);
% hold on;
% for i = 1:numel(alpha_f_values)
%     plot(time_values{i}, flightAngle_profiles{i}, 'LineWidth', 1.5, ...
%         'DisplayName', sprintf("\\alpha_f = %.0f^\\circ", alpha_f_values(i)));
% end
% xlabel('Time (s)');
% ylabel("\gamma (deg)");
% legend('show');
% grid on;
% box on;
% hold off;

% fig5 = figure(5);
% hold on;
% for i = 1:numel(alpha_f_values)
%     plot(time_values1{i}, acceleration_profiles{i}, 'LineWidth', 1.5, ...
%     'DisplayName', sprintf("\\alpha_f =%.0f^\\circ", alpha_f_values(i)));
% end
% xlabel('Time (s)'); % Label for x-axis
% ylabel("a\_{M} (m/s^2)"); % Label for y-axis
% legend('show'); % Display the legend
% grid on;
% box on;
% hold off;

% fig6 = figure(6);
% hold on;
% for i = 1:numel(alpha_f_values)
%     plot(time_values{i}, headingAngle_profiles{i}, 'LineWidth', 1.5, ...
%         'DisplayName', sprintf("\\alpha_f = %.0f^\\circ", alpha_f_values(i)));
% end
% xlabel('Time (s)');
% ylabel("\psi (deg)");
% legend('show');
% grid on;
% box on;
% hold off;
% end


