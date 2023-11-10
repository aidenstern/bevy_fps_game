//! A basic implementation of a character controller for a dynamic rigid body.
//!
//! This showcases the following:
//!
//! - Basic directional movement and jumping
//! - Support for both keyboard and gamepad input
//! - A configurable maximum slope angle for jumping
//! - Loading a platformer environment from a glTF
//!
//! The character controller logic is contained within the `plugin` module.
//!
//! For a kinematic character controller, see the `kinematic_character_3d` example.

mod plugin;

use bevy::prelude::*;
use bevy_fps_game::{
    controllers::fps::{
        CharacterCameraBundle, CharacterCameraPlugin, CharacterController,
        CharacterControllerBundle, CharacterControllerPlugin,
    },
    LookTransformPlugin,
};
use bevy_xpbd_3d::{math::*, prelude::*};
use plugin::*;

fn main() {
    App::new()
        .insert_resource(Msaa::Sample4)
        .add_plugins((
            DefaultPlugins,
            EmbeddedAssetsPlugin,
            PhysicsPlugins::default(),
            LookTransformPlugin,
            CharacterCameraPlugin::default(),
        ))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    assets: Res<AssetServer>,
) {
    // Logical Player
    commands.spawn(Camera3dBundle::default()).insert((
        // PbrBundle {
        //     mesh: meshes.add(Mesh::from(shape::Capsule {
        //         radius: 0.4,
        //         ..default()
        //     })),
        //     material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
        //     transform: Transform::from_xyz(0.0, 1.5, 0.0),
        //     ..default()
        // },
        CharacterCameraBundle::new(
            CharacterControllerBundle::new(Collider::capsule(1.0, 0.4)).with_movement(
                30.0,
                0.92,
                7.0,
                (30.0 as Scalar).to_radians(),
            ),
            Vec3::new(-2.0, 5.0, 5.0),
            Vec3::new(0., 0., 0.),
            Vec3::Y,
        ),
        Friction::ZERO.with_combine_rule(CoefficientCombine::Min),
        Restitution::ZERO.with_combine_rule(CoefficientCombine::Min),
        GravityScale(2.0),
    ));

    // A cube to move around
    commands.spawn((
        RigidBody::Dynamic,
        Collider::cuboid(1.0, 1.0, 1.0),
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
            material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
            transform: Transform::from_xyz(3.0, 2.0, 3.0),
            ..default()
        },
    ));

    // Environment (see `async_colliders` example for creating colliders from scenes)
    commands.spawn((
        SceneBundle {
            scene: assets.load("embedded://character_controller_demo.glb#Scene0"),
            transform: Transform::from_rotation(Quat::from_rotation_y(-std::f32::consts::PI * 0.5)),
            ..default()
        },
        AsyncSceneCollider::new(Some(ComputedCollider::ConvexHull)),
        RigidBody::Static,
    ));

    // Light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 6000.0,
            range: 50.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(0.0, 15.0, 0.0),
        ..default()
    });
}
