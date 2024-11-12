function animate_stl(pos, q)

global DART_stl orientation_plot orientation_plot_handle orientation_animation trajectory_plot trajectory_plot_handle trajectory_animation
global pos_tracked_X pos_tracked_Y pos_tracked_Z

% Quaternion representing inertial-to-body coordinate transformation
q_I2B = q;

% Update inertial position vectors
pos_tracked_X = [pos_tracked_X pos(1)];
pos_tracked_Y = [pos_tracked_Y pos(2)];
pos_tracked_Z = [pos_tracked_Z pos(3)];

% Update STL vertices (rotation)
DART_stl.body_vertices = quatrotate(quaternion(q_I2B), DART_stl.inertial_vertices);
DART_stl.body_vertices_X = pos(1) + DART_stl.body_vertices(:,1);
DART_stl.body_vertices_Y = pos(2) + DART_stl.body_vertices(:,2);
DART_stl.body_vertices_Z = pos(3) + DART_stl.body_vertices(:,3);

% Refresh live plots
refreshdata(orientation_plot)
refreshdata(trajectory_plot)

% Update orientation plot animation
orientation_frame = getframe(orientation_plot_handle);
writeVideo(orientation_animation, orientation_frame);

% Update trajectory plot animation
trajectory_frame = getframe(trajectory_plot_handle);
writeVideo(trajectory_animation, trajectory_frame);

end