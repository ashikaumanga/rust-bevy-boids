use bevy::math::Vec3;
use bevy_rapier3d::{plugin::RapierContext, prelude::QueryFilter};

//Generates a set of direction vectors distributed on a sphere within a specified visible angle.
pub fn generate_directions(points_on_sphere: u16, visible_angle: f32) -> Vec<Vec3> {
    let mut directions = Vec::new();
    let golden_ratio = (1.0 + 5.0_f32.sqrt()) / 2.0;
    let angle_increment = std::f32::consts::PI * 2.0 * golden_ratio;
    let cos_threshold = (visible_angle.to_radians() / 2.0).cos(); // Cosine of half FOV

    for points in 0..points_on_sphere {
        let t = points as f32 / points_on_sphere as f32;
        let inclination = (1.0 - 2.0 * t).acos();
        let azimuth = angle_increment * points as f32;
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

//Returns the first unobstructed direction from a set of directions.
pub fn unobstructed_dir<'a>(conrext: &RapierContext, look_dirs: &'a Vec<Vec3>, location: &Vec3, forward: &'a Vec3) -> &'a Vec3 {
for dir in look_dirs.iter() {
    let hit = conrext.cast_ray(*location, *dir, 4.0, true, QueryFilter::only_fixed());
    if hit.is_none() {
        return dir;
    }
}
forward
}
