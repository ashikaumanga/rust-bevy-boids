mod ray_casting_utils;
mod mesh_utils;

use bevy::{color::palettes::css::WHITE, prelude::*, text::FontSmoothing, utils::HashMap};
use bevy_dev_tools::fps_overlay::{FpsOverlayConfig, FpsOverlayPlugin};
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use bevy_rapier3d::{plugin::{NoUserData, RapierPhysicsPlugin, ReadRapierContext}, prelude::{Collider, ComputedColliderShape, QueryFilter, RigidBody, TriMeshFlags}};

use std::f32::consts::PI;

#[derive(Component)]
struct Boid {
    position: Vec3,
    velocity: Vec3,
}

#[derive(Component)]
struct Target {
    position : Vec3,
}

#[derive(Resource)]
struct BoidSettings {
   count : u16,
   raycasy_directions : Vec<Vec3>,
   //forces
   cohesion_force:   f32,
   separation_force: f32,
   alignment_force: f32,
   //distances
   separation_distance: f32,
   alignment_distance: f32,
   cohesion_distance: f32,
   max_speed: f32,
   follow_target: bool,
   selected_distance: SelectedDistance
   //sboids : Vec<&Boid>,
}
enum SelectedDistance {
    None,
    Separation,
    Alignment,
    Cohesion,
}

const CAGE_SIZE: f32 = 20.0;

fn main() {
    App::new()
    .add_plugins(DefaultPlugins{})
    .add_plugins(            FpsOverlayPlugin {
        config: FpsOverlayConfig {
            text_config: TextFont {
                font_size: 12.0,
                font: default(),
                font_smoothing: FontSmoothing::default(),
            },
            text_color: Color::WHITE,
            enabled: true,
        },
    },)
    .add_plugins(EguiPlugin)
    .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
    .insert_resource(BoidSettings{ 
        count: 500,
        raycasy_directions: ray_casting_utils::generate_directions(1500, 270.0),
        cohesion_force: 0.02,
        separation_force: 0.08,
        alignment_force: 0.06,
        separation_distance: 1.0,
        alignment_distance: 1.0,
        cohesion_distance: 1.5,
        max_speed: 4.0,
        follow_target: true,
        selected_distance: SelectedDistance::None
    })
    .add_plugins(PanOrbitCameraPlugin)
    .add_systems(Startup, setup_scene)
    .add_systems(Update, (draw_selected_distance_arrow,ui_update,update_target_mesh,updates_boids_mesh,update_boid_physics) )
    .run();

}

fn setup_scene(mut commands: Commands,
        mut meshes: ResMut<Assets<Mesh>>,
        mut materials: ResMut<Assets<StandardMaterial>>,
        context : ResMut<BoidSettings>,
       ) {
    commands.spawn(Target {
        position: Vec3::new(0.0, 0.0, 0.0),
    }).insert((Mesh3d(meshes.add(Sphere::new(0.3))),
                      MeshMaterial3d(materials.add(Color::linear_rgba(1.0, 0.0, 0.0, 1.0))),
                      Transform::from_xyz(0.0, 0.3, 0.0)));

    let boid_mesh = Mesh3d(meshes.add(Cone::new(0.1, 0.3)));
    let boid_mat = MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255)));
 
    for i in 0..context.count {
        let x = i as f32 * 2.0;
        let y = 0.0;
        let z = ((i % 4)) as f32;
        let boid = Boid {
            position: Vec3::new(x.sin()*2.0, x.cos()*2.0, z),
            velocity: Vec3::ZERO,
        };
        commands.spawn(boid)
        .insert((
            boid_mesh.clone(),
            boid_mat.clone(),
            Transform::from_xyz(x, y, z),
        ));   
    }
    

// cage wireframe
   let mesh = mesh_utils::create_wireframe_cube(CAGE_SIZE, CAGE_SIZE, CAGE_SIZE);
   commands.spawn((
   Mesh3d(meshes.add(mesh)),
   MeshMaterial3d(materials.add(StandardMaterial {
        base_color: WHITE.into(),
        unlit : true,
        ..Default::default()
    })),
     Transform::from_xyz(0.0, 0.0, 0.0),
));

//obstacles with physics engine config
//torus
let obsticle_colors = [Color::srgb_u8(255, 198, 174),
                          Color::srgb_u8(186, 156, 202),
                          Color::srgb_u8(115, 143, 182)];

for i in 0..3 {
   let t_mesh = meshes.add(Torus::new(2.0+(i as f32 *2.0), 3.0+(i as f32*2.0) ));
   let t_mesh_collider= Collider::from_bevy_mesh(&meshes.get(t_mesh.id()).unwrap(), &ComputedColliderShape::TriMesh(TriMeshFlags::all())).unwrap();
   
   commands.spawn(RigidBody::Fixed)
        .insert(t_mesh_collider)
        .insert(Mesh3d(t_mesh))
        .insert(MeshMaterial3d(materials.add(obsticle_colors[i % 3])))
        .insert(Transform::from_xyz(0.0, -4.0*i as f32, 0.0));
}

//sphere
let t_mesh = meshes.add(Sphere::new(3.0));
let t_mesh_collider= Collider::from_bevy_mesh(&meshes.get(t_mesh.id()).unwrap(), &ComputedColliderShape::TriMesh(TriMeshFlags::all())).unwrap();

commands.spawn(RigidBody::Fixed)
     .insert(t_mesh_collider)
     .insert(Mesh3d(t_mesh))
     .insert(MeshMaterial3d(materials.add(Color::srgb(0.0, 0.0, 1.0))))
     .insert(Transform::from_xyz(0.0, 2.0, -5.0));



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

fn ui_update(mut contexts: EguiContexts,
    mut boid_config : ResMut<BoidSettings>) {
        
    egui::Window::new("Bevy Boids").max_width(140.0).show(contexts.ctx_mut(), |ui| {
        ui.add(egui::Slider::new(&mut boid_config.cohesion_force, 0.00..=0.20).text("cohesion"));
        ui.add(egui::Slider::new(&mut boid_config.alignment_force, 0.00..=0.20).text("alignment"));
        ui.add(egui::Slider::new(&mut boid_config.separation_force, 0.00..=0.20).text("seperation"));
        ui.add(egui::Separator::default());
        let res_coh = ui.add(egui::Slider::new(&mut boid_config.cohesion_distance, 0.00..=2.00).text("cohesion distance"));
        let res_ali = ui.add(egui::Slider::new(&mut boid_config.alignment_distance, 0.00..=3.00).text("alignment distance"));
        let res_sep = ui.add(egui::Slider::new(&mut boid_config.separation_distance, 0.00..=4.00).text("seperation distance"));
        ui.add(egui::Separator::default());
        ui.add(egui::Slider::new(&mut boid_config.max_speed, 0.50..=10.00).text("max speed"));
        ui.add(egui::Checkbox::new(&mut boid_config.follow_target, "follow target"));
        boid_config.selected_distance = SelectedDistance::None;
        if res_coh.hovered() {
            //info!("Cohesion distance hovered");
            boid_config.selected_distance = SelectedDistance::Cohesion;
        }
        if res_ali.hovered() {
            //info!("Aligment distance hovered");
            boid_config.selected_distance = SelectedDistance::Alignment;
        }
        if res_sep.hovered() {
            //info!("Seperated distance hovered");
            boid_config.selected_distance = SelectedDistance::Separation;
        }
        
    });
}

fn updates_boids_mesh(
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

fn update_target_mesh(
    mut query: Query<(&mut Target, &mut Transform)>,
    time: Res<Time>
) {
    let t=time.elapsed().as_secs_f32();
    for (mut target, mut transform) in query.iter_mut() {
        target.position = Vec3::new(t.sin()*5.0, t.cos()*5.0, t.sin()*3.0 * t.cos() );
        transform.translation = target.position;
    }
}

//
fn draw_selected_distance_arrow(mut gizmos: Gizmos,
    query_enemies: Query<(Entity, &mut Boid, &mut Transform)>,
    settings : ResMut<BoidSettings>) {
    let selected_d = &settings.selected_distance;

    let first_enemy = query_enemies.iter().next().unwrap();
    match selected_d{
        SelectedDistance::Separation => {
            gizmos.arrow(first_enemy.1.position, 
                first_enemy.1.position + first_enemy.1.velocity.normalize() * settings.separation_distance,  
                Color::srgb(1.0, 0.0, 0.0));
        }
        SelectedDistance::Alignment => {
            gizmos.arrow(first_enemy.1.position, 
                first_enemy.1.position + first_enemy.1.velocity.normalize() * settings.alignment_distance,  
                Color::srgb(0.0, 1.0, 0.0));
        } SelectedDistance::Cohesion => {
            gizmos.arrow(first_enemy.1.position, 
                first_enemy.1.position + first_enemy.1.velocity.normalize() * settings.cohesion_distance,  
                Color::srgb(1.0, 1.0, 0.0));
        }
        
        _ => {},
    }
    
}

fn update_boid_physics(mut query_enemies: Query<(Entity, &mut Boid, &mut Transform)>,
    query_target: Query<&Target>,
    rapier_context: ReadRapierContext,
    time: Res<Time>,
    settings : ResMut<BoidSettings>
) {
    let rapier = rapier_context.single();
    //distances
    let separation_distance = settings.separation_distance;
    let alignment_distance = settings.alignment_distance;
    let cohesion_distance = settings.cohesion_distance;
    //forces
    let cohesion_force = settings.cohesion_force;
    let seperation_force = settings.separation_force;
    let alignment_force = settings.alignment_force;
    //max speed
    let max_speed = settings.max_speed;
    let boundary_distance = 0.5;
    let boundary_force_strength = 0.5;
    
    let target = query_target.single();
    let fleet_target = target.position;
    

    struct Body {
        position: Vec3,
        velocity: Vec3,
    }

    let mut body_map: HashMap<u32, Body> = HashMap::new();
    for (entity, boid,  _) in query_enemies.iter() {
        body_map.insert(entity.index(), Body {
            position: boid.position,
            velocity: boid.velocity,
        });
    }
    
    for (entity, boid,  _) in query_enemies.iter() {
        let mut separation = Vec3::ZERO;
        let mut alignment = Vec3::ZERO;
        let mut cohesion = Vec3::ZERO;
        let mut boundary_force = Vec3::ZERO;
        let mut fleet_force;
        let mut neighbor_count = 0;

        for (other_entity, other_boid,  _) in query_enemies.iter() {
            //if other_boid.flock_id != boid.flock_id {
             //   continue;
            //}
            if entity.index() != other_entity.index() {
                let distance = boid.position.distance(other_boid.position);

                if distance < separation_distance && distance > 0.0 {
                    let diff = boid.position - other_boid.position;
                    separation += diff.normalize() / distance;
                    neighbor_count += 1;
                }

                if distance < alignment_distance {
                    alignment += other_boid.velocity;
                    neighbor_count += 1;
                }

                if distance < cohesion_distance {
                    cohesion += other_boid.position;
                    neighbor_count += 1;
                }
            }
        }

        if neighbor_count > 0 {
            separation /= neighbor_count as f32;
            alignment /= neighbor_count as f32;
            cohesion /= neighbor_count as f32;

            cohesion = cohesion - boid.position;
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
        if let Some((_,_toi)) = hit {
                let free_dir = ray_casting_utils::unobstructed_dir(&rapier, &settings.raycasy_directions, &boid.position, &boid.velocity);
                separation += free_dir;    
        }

        let tmp = body_map.get_mut(&entity.index()).unwrap();
        tmp.velocity +=  (separation) + (alignment + cohesion) + boundary_force;
        if settings.follow_target {
            tmp.velocity += fleet_force;
        }
        tmp.velocity = tmp.velocity.clamp_length_max(max_speed);
        tmp.position += tmp.velocity * time.delta_secs();

    }

    for (entity, mut boid,   _) in query_enemies.iter_mut() {
        let tmp =body_map.get(&entity.index()).unwrap();
        boid.position = tmp.position;
        boid.velocity = tmp.velocity;
        
    }
}
