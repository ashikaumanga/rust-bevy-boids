use bevy::{asset::RenderAssetUsages, color::palettes::{basic, css::{RED, SILVER, WHITE}}, diagnostic::FrameTimeDiagnosticsPlugin, log, pbr::wireframe::{Wireframe, WireframeConfig, WireframePlugin}, prelude::*, render::{mesh::{self, Indices, PrimitiveTopology, VertexAttributeValues}, settings::{RenderCreation, WgpuFeatures, WgpuSettings}, RenderPlugin}, scene::ron::de, text::FontSmoothing, utils::{info, HashMap}};
use bevy_dev_tools::fps_overlay::{FpsOverlayConfig, FpsOverlayPlugin};
//use bevy_rts_camera::{RtsCamera, RtsCameraControls, RtsCameraPlugin}
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use bevy_rapier3d::{plugin::{NoUserData, RapierContext, RapierPhysicsPlugin, ReadRapierContext}, prelude::{Collider, ComputedColliderShape, QueryFilter, Restitution, RigidBody, TriMeshFlags}, render::{ColliderDebugColor, RapierDebugRenderPlugin}};


use std::f32::consts::PI;


#[derive(Component)]
struct Boid {
    position: Vec3,
    velocity: Vec3,
    flock_id : u16
}

#[derive(Component)]
struct Target {
    position : Vec3,
}

#[derive(Resource)]
struct BoidContext {
   count : u8,
   directions : Vec<Vec3>,
   //sboids : Vec<&Boid>,
}

const CAGE_SIZE: f32 = 20.0;


const NUM_VIEW_DIRECTIONS: usize = 100; // Number of sample directions

 

pub fn generate_directions(visible_angle: f32) -> Vec<Vec3> {

    let mut directions = Vec::new();

    let golden_ratio = (1.0 + 5.0_f32.sqrt()) / 2.0;

    let angle_increment = std::f32::consts::PI * 2.0 * golden_ratio;

    let cos_threshold = (visible_angle.to_radians() / 2.0).cos(); // Cosine of half FOV

 

    for i in 0..NUM_VIEW_DIRECTIONS {

        let t = i as f32 / NUM_VIEW_DIRECTIONS as f32;

        let inclination = (1.0 - 2.0 * t).acos();

        let azimuth = angle_increment * i as f32;

 

        let x = inclination.sin() * azimuth.cos();

        let y = inclination.sin() * azimuth.sin();

        let z = inclination.cos();

        let dir = Vec3::new(x, y, z);

 

        // Only keep directions within the field of view

        if dir.dot(Vec3::Z) >= cos_threshold {

            directions.push(dir);

        }

    }
    directions

}


fn create_wireframe_cube(x_length: f32, y_length: f32, z_length: f32) -> Mesh {
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


fn main() {
    App::new()
    .add_plugins(DefaultPlugins{})
    .add_plugins(            FpsOverlayPlugin {
        config: FpsOverlayConfig {
            text_config: TextFont {
                // Here we define size of our overlay
                font_size: 12.0,
                // If we want, we can use a custom font
                font: default(),
                // We could also disable font smoothing,
                font_smoothing: FontSmoothing::default(),
            },
            // We can also change color of the overlay
            text_color: Color::WHITE,
            enabled: true,
        },
    },)
    
    .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
    .add_plugins(RapierDebugRenderPlugin::default())
    .insert_resource(BoidContext{ count: 150, directions: generate_directions(180.0)})
    .add_plugins(PanOrbitCameraPlugin)
    .add_systems(Startup, setup_scene)
    .add_systems(Update, (update_target,sync_enemy_mesh_transform,enemy_ai) )
    .run();
}

fn setup_scene(mut commands: Commands,
        mut meshes: ResMut<Assets<Mesh>>,
        mut materials: ResMut<Assets<StandardMaterial>>,
        context : ResMut<BoidContext>,
       ) {

    commands.spawn(Target {
        position: Vec3::new(0.0, 0.0, 0.0),
    }).insert((Mesh3d(meshes.add(Cuboid::new(0.3, 0.3,0.3))),
                      MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 155))),
                      Transform::from_xyz(0.0, 0.5, 0.0)));

    let boid_mesh = Mesh3d(meshes.add(Cone::new(0.1, 0.3)));
    let boid_mat = MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255)));
 
    for i in 0..context.count {
        let x = i as f32 * 2.0;
        let y = 0.0;
        let z = ((i % 4)) as f32;
        let boid = Boid {
            position: Vec3::new(x.sin()*2.0, x.cos()*2.0, z),
            velocity: Vec3::ZERO,
            flock_id: (i % 3) as u16,
        };
        //context.boids.push(boid);
        commands.spawn(boid)
        .insert((
            boid_mesh.clone(),
            boid_mat.clone(),
            Transform::from_xyz(x, y, z),
        ));
        //.insert(RigidBody::KinematicPositionBased)
        //.insert(Collider::cone(0.3/2.0,0.1))
        //.insert(Restitution::coefficient(0.7));
        
    }
    

    // cube wireframe
   let mesh = create_wireframe_cube(CAGE_SIZE, CAGE_SIZE, CAGE_SIZE);
   commands.spawn((
    Mesh3d(meshes.add(mesh)),
    MeshMaterial3d(materials.add(StandardMaterial {
        base_color: WHITE.into(),
        unlit : true,
        ..Default::default()
    })),
     Transform::from_xyz(0.0, 0.0, 0.0),
));

   //physics
   for i in 0..3 {

   
   let t_mesh = meshes.add(Torus::new(2.0+i as f32, 3.0+i as f32));
   let col= Collider::from_bevy_mesh(&meshes.get(t_mesh.id()).unwrap(), &ComputedColliderShape::TriMesh(TriMeshFlags::all())).unwrap();
   
   commands.spawn(RigidBody::Fixed)
        .insert(col)
        .insert(Mesh3d(t_mesh))
        .insert(MeshMaterial3d(materials.add(Color::WHITE)))
        .insert(Transform::from_xyz(0.0, -4.0*i as f32, 0.0));

   }


    // lights
    // directional 'sun' light
    commands.spawn((
        DirectionalLight {
            illuminance: light_consts::lux::OVERCAST_DAY,
            shadows_enabled: true,
            ..default()
        },
        Transform {
            translation: Vec3::new(0.0, 2.0, 0.0),
            rotation: Quat::from_rotation_x(-PI / 4.),
            ..default()
        }));

    commands.spawn((
        PointLight {
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0),
    ));
    
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 10.0, 20.0).looking_at(Vec3::ZERO, Vec3::Y),
        PanOrbitCamera::default()
    ));

}

fn sync_enemy_mesh_transform(
    mut query: Query<(&Boid, &mut Transform)>,
) {
    for (boid, mut transform) in query.iter_mut() {
        transform.translation = boid.position;
        let up = Vec3::Y;
        let forward = boid.velocity.normalize();
        let rotation = Quat::from_rotation_arc(up, forward);
        transform.rotation = rotation;
    }
}

fn update_target(
    mut query: Query<(&mut Target, &mut Transform)>,
    time: Res<Time>
) {
    let t=time.elapsed().as_secs_f32();
    for (mut target, mut transform) in query.iter_mut() {
        target.position = Vec3::new(t.sin()*5.0, t.cos()*5.0, t.sin()*3.0 * t.cos() );
        //target.
        transform.translation = target.position;
    }
}

fn enemy_ai(mut commands: Commands,
    mut query: Query<(Entity, &mut Boid, &mut Transform)>,
    query2: Query<&Target>,
    rapier_context: ReadRapierContext,
    time: Res<Time>,
    context : ResMut<BoidContext>
) {
    let rapier = rapier_context.single();
    let separation_distance = 1.0;
    let alignment_distance = 1.0;
    let cohesion_distance = 1.5;
    let cohesion_force = 0.02;
    let seperation_force = 0.08;
    let alignment_force = 0.06;
    let max_speed = 4.0;
    let boundary_distance = 0.5;
    let boundary_force_strength = 0.5;
    let mut fleet_target = Vec3::ZERO;

    for t in query2.iter() {
        fleet_target = t.position;
    }

    struct Body {
        position: Vec3,
        velocity: Vec3,
    }

    let mut body_map: HashMap<u32, Body> = HashMap::new();
    for (entity, boid,  _) in query.iter() {
        body_map.insert(entity.index(), Body {
            position: boid.position,
            velocity: boid.velocity,
        });
    }
    

    for (entity, boid,  _) in query.iter() {
        let mut separation = Vec3::ZERO;
        let mut alignment = Vec3::ZERO;
        let mut cohesion = Vec3::ZERO;
        let mut boundary_force = Vec3::ZERO;
        let mut fleet_force;
        let mut count = 0;

        for (other_entity, other_boid,  _) in query.iter() {
            //if other_boid.flock_id != boid.flock_id {
             //   continue;
            //}
            if entity.index() != other_entity.index() {
                let distance = boid.position.distance(other_boid.position);

                if distance < separation_distance && distance > 0.0 {
                    let diff = boid.position - other_boid.position;
                    separation += diff.normalize() / distance;
                }

                if distance < alignment_distance {
                    alignment += other_boid.velocity;
                }

                if distance < cohesion_distance {
                    cohesion += other_boid.position;
                }

                count += 1;
            }
        }

        if count > 0 {
            separation /= count as f32;
            alignment /= count as f32;
            cohesion /= count as f32;

            cohesion = (cohesion / count as f32) - boid.position;
            if cohesion.length() > 0.0 {
                cohesion = cohesion.normalize() * max_speed - boid.velocity;
                cohesion = cohesion.clamp_length_max(cohesion_force);
            }

            if alignment.length() > 0.0 {
                alignment = alignment.normalize() * max_speed - boid.velocity;
                alignment = alignment.clamp_length_max(alignment_force);
            }
            if separation.length()> 0.0 {

            separation = separation.normalize() * max_speed - boid.velocity;
            separation = separation.clamp_length_max(seperation_force);
        }
        }

                // Fleet force calculation
        fleet_force = fleet_target - boid.position;
        if fleet_force.length() > 0.0 {
            fleet_force = fleet_force.normalize() * max_speed - boid.velocity;
            fleet_force = fleet_force.clamp_length_max(0.09);
        }
                let half_cage = CAGE_SIZE/2.0;
                // Boundary force calculation
                if boid.position.x > half_cage - boundary_distance {
                    boundary_force.x -= boundary_force_strength;
                } else if boid.position.x < -half_cage + boundary_distance {
                    boundary_force.x += boundary_force_strength;
                }
                if boid.position.y > half_cage - boundary_distance {
                    boundary_force.y -= boundary_force_strength;
                } else if boid.position.y < -half_cage + boundary_distance {
                    boundary_force.y += boundary_force_strength;
                }
                if boid.position.z > half_cage - boundary_distance {
                    boundary_force.z -= boundary_force_strength;
                } else if boid.position.z < -half_cage + boundary_distance {
                    boundary_force.z += boundary_force_strength;
                }
        //ray casting

        //let free_dir = boid.velocity;

        //do we collide
        let hit = rapier.cast_ray(boid.position,boid.velocity, 4.0, true, QueryFilter::only_fixed());
        if let Some((entity,_toi)) = hit {

                let free_dir = unobstructed_dir(&rapier, &context.directions, &boid.position, &boid.velocity);
                separation += separation + free_dir;
            
        }

        let tmp = body_map.get_mut(&entity.index()).unwrap();
        tmp.velocity +=  (separation) + (alignment + cohesion) + boundary_force + fleet_force;
        tmp.velocity = tmp.velocity.clamp_length_max(max_speed);
        tmp.position += tmp.velocity * time.delta_secs();

    }

    for (entity, mut boid,   _) in query.iter_mut() {
        let tmp =body_map.get(&entity.index()).unwrap();
        boid.position = tmp.position;
        boid.velocity = tmp.velocity;
        
    }
}

fn unobstructed_dir(conrext: &RapierContext, look_dirs: &Vec<Vec3>, location: &Vec3, forward: &Vec3) -> Vec3 {
    for dir in look_dirs.iter() {
        let hit = conrext.cast_ray(*location, *dir, 4.0, true, QueryFilter::only_fixed());
        if hit.is_none() {
            return *dir;
        }
    }
    *forward
}
