% This file produces plots for the results of simulations produced by means
% of PyChrono and stored under the folder_1 

% Sanjana Sanjay Dhulla
% Mattia Gramuglia
% Andrea L'Afflitto

% 09/21/2023

clear
clc
close all

%% Initializing the folder
% Name of the folder containing the .csv files
folder_1= 'my_ball_density_MRACwithBASELINE';
% folder_1= 'my_ball_density_MRACwithBaseline';

% Scan the folder to get the file names 
data = dir(fullfile('my_ball_density_MRACwithBASELINE\*.csv'));

%% Iterating through the files
% Initialize an empty cell array to store the names
file_name_csv = cell(1, numel(data));

% Loop through the elements and extract the 'name' field
for ii=1:numel(data)
    file_name_csv{ii} = data(ii).name;
end

% Display the list of names
% disp(file_name_csv);

%% Extracting the required position error data from each file and taking the L2-norm, 
% and extracting densities from the file name
norm_tracking_error_at_some_density = zeros(size(data));
Total_thrust_at_some_density = zeros(size(data));
density = zeros(numel(data),1);
computation_time = zeros(size(data));
% Iterating through each file
for ii=1:numel(data)
    % Reading the required potion error and time data from each file
    file_name  = folder_1 + "\" + file_name_csv{ii};
    
    % For versions of MATLAB older than 2020, readlines command cannot be used
    % to read headers, hence the below code needs to be implemented to extract
    % controller type and density
    fileID = fopen(file_name, 'r');

    % Read the header lines (assuming there are three header lines)
    headerLines = cell(3, 1);
    for i = 1:3
        headerLines{i} = fgetl(fileID);
    end
    dataTable = cell2table(cellfun(@(x) strsplit(x, ','), headerLines, 'UniformOutput', false));
    % Extracting controller name and ball density
    controller_type = dataTable.Var1{2}{2};
    density(ii,1) = str2double(dataTable.Var1{3}{3});
   
    Wrapper_data = readtable(file_name);
    Wrapper_data = table2array(Wrapper_data);
    
    % Deleting the heading line
    Wrapper_data = Wrapper_data(4:end,:);
    
    % Extracting simulation time vector
    simulation_time_vector = Wrapper_data(:,1);
    % Convert the cell array of strings to a numeric vector
    simulation_time_vector = str2double(simulation_time_vector);
    
    % Extracting computation time to determine the total time taken for the
    % simulation to run for a particular density
    computation_time_at_current_density = Wrapper_data(:,2);
    computation_time_at_current_density = str2double(computation_time_at_current_density);
    computation_time(ii) = computation_time_at_current_density(end-1,end);
    
    % Extracting translational_position_in_I and
    % translational_position_in_I_user and subtracting them to get position
    % error. The data vector of PID and MRAC is different hence checking
    % the controller and then extracting the required data
    translational_position_I = Wrapper_data(:,3:5);
    
    if contains(controller_type, 'MRAC')
        translational_position_I_user = Wrapper_data(:,33:35);
        Thrust = Wrapper_data(:,49:56);
        
    elseif strcmp(controller_type, 'PID')
        translational_position_I_user = Wrapper_data(:,24:26);
        Thrust = Wrapper_data(:,40:47);
    end
    
    % Determining the total thrust 
    Thrust = cellfun(@str2double, Thrust, 'UniformOutput', false);
    Thrust_mat = cell2mat(Thrust);
    Total_thrust = sum(Thrust_mat,2);
    
    %Computing position error
    translational_position_I = cellfun(@str2double, translational_position_I, 'UniformOutput', false);
    translational_position_I_user = cellfun(@str2double, translational_position_I_user, 'UniformOutput', false);

    translational_position_error = cellfun(@minus, translational_position_I, translational_position_I_user, 'UniformOutput', 0);
    %translational_position_error = translational_position_I(:,:,:) - translational_position_I_user(:,:,:);
    % Convert the cell array result to a numeric array
    translational_position_error = cell2mat(translational_position_error);
    norm_postion_error = zeros(length(translational_position_error),1);
    norm_t_sq = zeros(length(translational_position_error),1);
    for jj=1:length(translational_position_error)
        % Taking norm at each timestep
        norm_postion_error(jj,1) = norm(translational_position_error(jj,:));
        
        % Taking square of norm at each timestep
        norm_t_sq(jj,1) = norm_postion_error(jj,1)*norm_postion_error(jj,1);
    end
    
    % Integrating the norms using trapz(T,Norm) or cumtrapz(T,Norm)
    integral_norm = trapz(simulation_time_vector',norm_t_sq');
    
    % Assing norm of position error for each desnity to a variable (array)
    norm_tracking_error_at_some_density(ii,1) = sqrt(integral_norm);
    Total_thrust_at_some_density(ii,1) = trapz(simulation_time_vector',Total_thrust');
    
    message = [round(num2str(ii/numel(data)*100,4)),'% of data processed'];
    disp(message)
end

%% Creating a dictionary to pair the density and tracking error and sorting them in ascending order
% The map auto-sorts the keys and values with respect to the keys
mapObj = containers.Map(density,norm_tracking_error_at_some_density);

% Converting the sorted values into array
densities = cell2mat(keys(mapObj));
tracking_L2_error_norm = cell2mat(values(mapObj));



% Following the same as above for Thrust
mapObj = containers.Map(density,Total_thrust_at_some_density);
Total_thrust_every_density = cell2mat(values(mapObj));


% Same for computation time
mapObj = containers.Map(density,computation_time);
computation_time = cell2mat(values(mapObj));
%% Interpolation of data
% Use an interpolating polynomial
order_interpolation = 3;
coefficients_L2_error_norm_vs_density = polyfit(densities, tracking_L2_error_norm,order_interpolation);
interpolated_L2_error_norm = polyval(coefficients_L2_error_norm_vs_density, densities);



order_interpolation = 3;
coefficients_Total_thrust_vs_density = polyfit(densities, Total_thrust_every_density,order_interpolation);
interpolated_Total_thrust = polyval(coefficients_Total_thrust_vs_density, densities);

%% Plotting
% Tracking error vs density
set(figure,'Color','white','WindowState','maximized')
plot(densities, tracking_L2_error_norm, '-o','LineWidth',2)
hold on
plot(densities,interpolated_L2_error_norm,'k-','LineWidth',2)
xlabel('Ball density','interpreter','latex','fontsize',22)
ylabel('$\Vert e(\cdot) \Vert_{L_2}$','interpreter','latex','fontsize',22)
l = legend('Raw data',['Interpolating polynomial order ', num2str(order_interpolation)]);
set(l,'interpreter','latex','fontsize',22)
axis tight

variable_title = ['Control technique:',' ', controller_type];
% variable_title = [variable_title,techique_used];

title(variable_title,'interpreter','latex','fontsize',22)




% Thrust vs density
set(figure,'Color','white','WindowState','maximized')
plot(densities, Total_thrust_every_density, '-o','LineWidth',2)
hold on
plot(densities,interpolated_Total_thrust,'k-','LineWidth',2)
xlabel('Ball density','interpreter','latex','fontsize',22)
ylabel('Total Thrust','interpreter','latex','fontsize',22)
l = legend('Raw data',['Interpolating polynomial order ', num2str(order_interpolation)]);
set(l,'interpreter','latex','fontsize',22)
axis tight

variable_title = ['Control technique:',' ', controller_type];
% variable_title = [variable_title,techique_used];

title(variable_title,'interpreter','latex','fontsize',22)




% Computation time vs density
set(figure,'Color','white','WindowState','maximized')
plot(densities, computation_time, '-o','LineWidth',2)
xlabel('Ball density','interpreter','latex','fontsize',22)
ylabel('Computation Time [s]','interpreter','latex','fontsize',22)

axis tight

variable_title = ['Control technique:',' ', controller_type];
% variable_title = [variable_title,techique_used];

title(variable_title,'interpreter','latex','fontsize',22)