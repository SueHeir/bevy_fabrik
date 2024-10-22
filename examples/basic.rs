use bevy::prelude::*;

use bevy_fabrik::*;

#[derive(Component)]
struct IkTarget;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(InverseKinematicsPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, move_target)
        .add_systems(Update, update_chain_target)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let joint_mesh = meshes.add(Capsule3d { radius: 0.1, half_length: 0.25 });
    let joint_material = materials.add(StandardMaterial::default());
    let root = commands.spawn(SpatialBundle {
        global_transform: GlobalTransform::from_translation(Vec3::new(0.0, -3.0, 0.0)),
        ..default()
    }).id();


    let chain_tail = spawn_chain(
        &mut commands,
        joint_mesh,
        joint_material,
        Vec3::new(0.0, -2.6, 0.0),
        Vec3::new(0.0, 0.4, 0.0),
        14,
        root,
    );

    commands.entity(chain_tail).insert(IkChain::new(15));

    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-2.5, 6.5, 9.0)
            .looking_at(Vec3::Y * 3.0, Vec3::Y),
        ..default()
    });

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Sphere { radius: 0.1 }),
            material: materials.add(Color::srgb(1.0, 0.0, 0.0)),
            ..default()
        },
        IkTarget,
    ));

    commands.spawn(DirectionalLightBundle {
        transform: Transform::from_xyz(50.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
        directional_light: DirectionalLight {
            illuminance: 1_500.,
            ..default()
        },
        ..default()
    });
}

fn move_target(
    time: Res<Time>,
    mut query: Query<&mut Transform, With<IkTarget>>,
) {
    for mut transform in &mut query.iter_mut() {
        let t = time.elapsed().as_millis() as f32;

        // Some random coefficients for target movement variety
        transform.translation.y = (t * 0.003 + std::f32::consts::FRAC_PI_3).sin() * 2.0 + 3.0;
        transform.translation.x = (t * 0.001).cos() * 2.0;
        transform.translation.z = (t * 0.002 + std::f32::consts::FRAC_PI_3).cos() * 2.0;
    }
}

fn update_chain_target(
    mut chain_query: Query<&mut IkChain>,
    target_query: Query<&GlobalTransform, With<IkTarget>>,
) {
    if let Ok(target) = target_query.get_single() {
        for mut chain in chain_query.iter_mut() {
            chain.target = target.translation();
        }
    }
}

// Recursively spawn a chain of entities for bones
fn spawn_chain(
    commands: &mut Commands,
    mesh: Handle<Mesh>,
    material: Handle<StandardMaterial>,
    bone_root: Vec3,
    bone_tail: Vec3,
    joint_count: usize,
    parent: Entity,
) -> Entity {
    // Last entity is a segment tail only, so it doesn't need a mesh
    if joint_count == 1 {
        let tail = commands.spawn(SpatialBundle {
            transform: Transform::from_translation(bone_tail),
            global_transform: GlobalTransform::from_translation(bone_root),
            ..default()
        }).id();

        commands.entity(parent).add_child(tail);
        return tail;
    }

    let joint = commands.spawn(PbrBundle {
        mesh: mesh.clone(),
        material: material.clone(),
        transform: Transform::from_translation(bone_tail),
        global_transform: GlobalTransform::from_translation(bone_root),
        ..default()
    }).id();

    commands.entity(parent).add_child(joint);

    spawn_chain(
        commands,
        mesh,
        material,
        bone_root + bone_tail,
        bone_tail,
        joint_count - 1,
        joint,
    )
}
    