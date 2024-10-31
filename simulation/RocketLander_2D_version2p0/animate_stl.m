function animate_stl(q)

global DART_stl
q_I2B = q; % quaternion representing inertial-2-body coordinate transformation

for ctr=1
    DART_stl.body_vertices = quatrotate(quaternion(q_I2B), DART_stl.inertial_vertices);
    DART_stl.body_vertices_X = DART_stl.body_vertices(:,1);
    DART_stl.body_vertices_Y = DART_stl.body_vertices(:,2);
    DART_stl.body_vertices_Z = DART_stl.body_vertices(:,3);
    refreshdata
    drawnow
end
end