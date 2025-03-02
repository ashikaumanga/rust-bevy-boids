use bevy::{math::Vec3, render::mesh::{Indices, Mesh, PrimitiveTopology}, utils::default};

pub fn create_wireframe_cube(x_length: f32, y_length: f32, z_length: f32) -> Mesh {
    let half_x = x_length / 2.0;
    let half_y = y_length / 2.0;
    let half_z = z_length / 2.0;

    let vertices: Vec<Vec3> = vec![
        Vec3::new(-half_x, -half_y, -half_z), // 0
        Vec3::new( half_x, -half_y, -half_z), // 1
        Vec3::new( half_x,  half_y, -half_z), // 2
        Vec3::new(-half_x,  half_y, -half_z), // 3
        Vec3::new(-half_x, -half_y,  half_z), // 4
        Vec3::new( half_x, -half_y,  half_z), // 5
        Vec3::new( half_x,  half_y,  half_z), // 6
        Vec3::new(-half_x,  half_y,  half_z), // 7
    ];

    let edges: Vec<(usize, usize)> = vec![
        (0, 1), (1, 2), (2, 3), (3, 0), // Bottom face
        (4, 5), (5, 6), (6, 7), (7, 4), // Top face
        (0, 4), (1, 5), (2, 6), (3, 7), // Vertical edges
    ];

    let mut line_positions: Vec<[f32; 3]> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();

    for (start, end) in edges {
        let start_idx = line_positions.len() as u32;
        line_positions.push(vertices[start].into());
        line_positions.push(vertices[end].into());
        indices.push(start_idx);
        indices.push(start_idx + 1);
    }

    
    let mut mesh = Mesh::new(PrimitiveTopology::LineList, default());
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, line_positions);
    mesh.insert_indices(Indices::U32(indices));
    mesh
}
