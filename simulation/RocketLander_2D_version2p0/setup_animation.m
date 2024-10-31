function setup_animation(q)

global DART_stl DART_plot

q_setup = eul2quat(deg2rad([0 -90 0]), "ZYX"); % quaternion for setup (align body axes with inertial axes)

DART_stl = stlread("DART STL.stl");

DART_stl.inertial_vertices = quatrotate(quaternion(q_setup), DART_stl.vertices);

inches_2_meters = 0.0254;
scale_factor = (36*inches_2_meters)/(max(max(DART_stl.inertial_vertices)));
DART_stl.inertial_vertices = DART_stl.inertial_vertices * scale_factor;

q_I2B = q;
DART_stl.body_vertices = quatrotate(quaternion(q_I2B), DART_stl.inertial_vertices);
DART_stl.body_vertices_X = DART_stl.body_vertices(:,1);
DART_stl.body_vertices_Y = DART_stl.body_vertices(:,2);
DART_stl.body_vertices_Z = DART_stl.body_vertices(:,3);

% figure
DART_plot = plot3(DART_stl.body_vertices_X, DART_stl.body_vertices_Y, DART_stl.body_vertices_Z);
DART_plot.XDataSource = "DART_stl.body_vertices_X";
DART_plot.YDataSource = "DART_stl.body_vertices_Y";
DART_plot.ZDataSource = "DART_stl.body_vertices_Z";
axis equal
grid minor

refreshdata
drawnow
end