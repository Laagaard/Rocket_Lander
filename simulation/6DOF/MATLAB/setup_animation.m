function setup_animation(q)

global DART_stl orientation_plot orientation_plot_handle trajectory_plot trajectory_plot_handle orientation_animation trajectory_animation 
global DesiredAltitude DesiredRange pos_tracked_X pos_tracked_Y pos_tracked_Z

% Check for existing orientation animation file
if (isfile("orientation_animation.mp4"))
    delete("orientation_animation.mp4")
end

% Check for existing trajectory animation file
if (isfile("trajectory_animation.mp4"))
    delete("trajectory_animation.mp4")
end

% Create & open orientation animation object
orientation_animation = VideoWriter("orientation_animation", "MPEG-4");
open(orientation_animation);

% Create & open trajectory animation object
trajectory_animation = VideoWriter("trajectory_animation", "MPEG-4");
open(trajectory_animation);

% Quaternion for setup (align body axes with inertial axes)
q_setup = eul2quat(deg2rad([0 90 0]), "ZYX");

% Import STL file of DART vehicle
DART_stl_imported = stlread("DART STL.STL");

% Rotate vertices to align with inertial axes
DART_stl.inertial_vertices = quatrotate(quaternion(q_setup), DART_stl_imported.vertices);

% Scale STL file to accurate dimensions
inches_2_meters = 0.0254;
scale_factor = (36*inches_2_meters)/(max(max(DART_stl.inertial_vertices)));
DART_stl.inertial_vertices = DART_stl.inertial_vertices * scale_factor;

DART_stl.inertial_vertices_X = DART_stl.inertial_vertices(:,1);
DART_stl.inertial_vertices_Y = DART_stl.inertial_vertices(:,2);
DART_stl.inertial_vertices_Z = DART_stl.inertial_vertices(:,3);

% Inertial position for trajectory plot
pos_tracked = zeros(3,1);
pos_tracked_X = pos_tracked(1,:);
pos_tracked_Y = pos_tracked(2,:);
pos_tracked_Z = pos_tracked(3,:);

% Quaternion representing inertial-to-body transformation
q_I2B = q;

% Rotate STL vertices from inertial coordinates to body coordinates
DART_stl.body_vertices = quatrotate(quaternion(q_I2B), DART_stl.inertial_vertices);
DART_stl.body_vertices_X = DART_stl.body_vertices(:,1);
DART_stl.body_vertices_Y = DART_stl.body_vertices(:,2);
DART_stl.body_vertices_Z = DART_stl.body_vertices(:,3);

% Orientation Plot
figure("Name", "Orientation Plot")
orientation_plot = plot3(DART_stl.body_vertices_X, DART_stl.body_vertices_Y, DART_stl.body_vertices_Z, '-');
axis equal
grid minor
orientation_plot_handle = gcf;
orientation_plot.XDataSource = "DART_stl.body_vertices_X";
orientation_plot.YDataSource = "DART_stl.body_vertices_Y";
orientation_plot.ZDataSource = "DART_stl.body_vertices_Z";

% Trajectory Plot
figure("Name", "Trajectory Plot")
trajectory_plot = plot3(pos_tracked_X, pos_tracked_Y, pos_tracked_Z, 'k--');
grid minor
trajectory_plot_axes = gca;
trajectory_plot_axes.XLim = [-1.1*DesiredRange 1.1*DesiredRange];
trajectory_plot_axes.YLim = [-1.1*DesiredRange 3.5*DesiredRange];
trajectory_plot_axes.ZLim = [0 2.5*DesiredAltitude];
trajectory_plot_handle = gcf;
trajectory_plot.XDataSource = "pos_tracked_X";
trajectory_plot.YDataSource = "pos_tracked_Y";
trajectory_plot.ZDataSource = "pos_tracked_Z";

end