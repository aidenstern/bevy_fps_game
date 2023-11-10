use crate::{LookAngles, LookTransform, LookTransformBundle, Smoother};
use bevy::{
    ecs::query::Has, input::mouse::MouseMotion,
    prelude::*,
};
use bevy_xpbd_3d::{math::*, prelude::*};
pub struct CharacterControllerPlugin;

#[derive(Default)]
pub struct CharacterCameraPlugin {
    pub override_input_system: bool,
}

impl CharacterCameraPlugin {
    pub fn new(override_input_system: bool) -> Self {
        Self {
            override_input_system,
        }
    }
}

#[derive(Bundle)]
pub struct CharacterCameraBundle {
    controller: CharacterControllerBundle,
    look_transform: LookTransformBundle,
    transform: Transform,
}

impl CharacterCameraBundle {
    pub fn new(controller: CharacterControllerBundle, eye: Vec3, target: Vec3, up: Vec3) -> Self {
        // Make sure the transform is consistent with the controller to start.
        let transform = Transform::from_translation(eye).looking_at(target, up);

        Self {
            controller,
            look_transform: LookTransformBundle {
                transform: LookTransform::new(eye, target, up),
                smoother: Smoother::new(0.0),
            },
            transform,
        }
    }
}

impl Plugin for CharacterCameraPlugin {
    fn build(&self, app: &mut App) {
        let app = app
            .add_systems(PreUpdate, on_controller_enabled_changed)
            .add_systems(Update, movement)
            .add_event::<ControlAction>();

        if !self.override_input_system {
            app.add_systems(Update, (mouse_input, keyboard_input));
        }
    }
}


impl Plugin for CharacterControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<ControlAction>().add_systems(
            Update,
            (
                update_grounded,
                apply_deferred,
                apply_movement_damping,
            )
                .chain(),
        );
    }
}

/// An event sent for a character controller input action.
#[derive(Event)]
pub enum ControlAction {
    Look(Vec2),
    Move(Vec3),
}

/// A marker component indicating that an entity is using a character controller.
#[derive(Component)]
pub struct CharacterController {
    pub enabled: bool,
    pub mouse_rotate_sensitivity: Vec2,
    pub translate_sensitivity: f32,
}

impl Default for CharacterController {
    fn default() -> Self {
        Self {
            enabled: true,
            mouse_rotate_sensitivity: Vec2::splat(0.2),
            translate_sensitivity: 2.0,
        }
    }
}

define_on_controller_enabled_changed!(CharacterController);

/// A marker component indicating that an entity is on the ground.
#[derive(Component)]
#[component(storage = "SparseSet")]
pub struct Grounded;

/// The acceleration used for character movement.
#[derive(Component)]
pub struct MovementAcceleration(Scalar);

/// The damping factor used for slowing down movement.
#[derive(Component)]
pub struct MovementDampingFactor(Scalar);

/// The strength of a jump.
#[derive(Component)]
pub struct JumpImpulse(Scalar);

/// The maximum angle a slope can have for a character controller
/// to be able to climb and jump. If the slope is steeper than this angle,
/// the character will slide down.
#[derive(Component)]
pub struct MaxSlopeAngle(Scalar);

/// A bundle that contains the components needed for a basic
/// kinematic character controller.
#[derive(Bundle)]
pub struct CharacterControllerBundle {
    character_controller: CharacterController,
    rigid_body: RigidBody,
    collider: Collider,
    ground_caster: ShapeCaster,
    locked_axes: LockedAxes,
    movement: MovementBundle,
}

/// A bundle that contains components for character movement.
#[derive(Bundle)]
pub struct MovementBundle {
    acceleration: MovementAcceleration,
    damping: MovementDampingFactor,
    jump_impulse: JumpImpulse,
    max_slope_angle: MaxSlopeAngle,
}

impl MovementBundle {
    pub const fn new(
        acceleration: Scalar,
        damping: Scalar,
        jump_impulse: Scalar,
        max_slope_angle: Scalar,
    ) -> Self {
        Self {
            acceleration: MovementAcceleration(acceleration),
            damping: MovementDampingFactor(damping),
            jump_impulse: JumpImpulse(jump_impulse),
            max_slope_angle: MaxSlopeAngle(max_slope_angle),
        }
    }
}

impl Default for MovementBundle {
    fn default() -> Self {
        Self::new(30.0, 0.9, 7.0, PI * 0.45)
    }
}

impl CharacterControllerBundle {
    pub fn new(collider: Collider) -> Self {
        // Create shape caster as a slightly smaller version of collider
        let mut caster_shape = collider.clone();
        caster_shape.set_scale(Vector::ONE * 0.99, 10);

        Self {
            character_controller: CharacterController::default(),
            rigid_body: RigidBody::Dynamic,
            collider,
            ground_caster: ShapeCaster::new(
                caster_shape,
                Vector::ZERO,
                Quaternion::default(),
                Vector::NEG_Y,
            )
            .with_max_time_of_impact(0.2),
            locked_axes: LockedAxes::ROTATION_LOCKED,
            movement: MovementBundle::default(),
        }
    }

    pub fn with_movement(
        mut self,
        acceleration: Scalar,
        damping: Scalar,
        jump_impulse: Scalar,
        max_slope_angle: Scalar,
    ) -> Self {
        self.movement = MovementBundle::new(acceleration, damping, jump_impulse, max_slope_angle);
        self
    }
}

/// Sends [`ControlAction`] events based on mouse input.
fn mouse_input(
    mut control_events: EventWriter<ControlAction>,
    mut mouse_motion_events: EventReader<MouseMotion>,
    controllers: Query<&CharacterController>,
) {
    // Can only control one camera at a time.
    let controller = if let Some(controller) = controllers.iter().find(|c| c.enabled) {
        controller
    } else {
        return;
    };
    let CharacterController {
        mouse_rotate_sensitivity,
        ..
    } = *controller;

    let mut cursor_delta = Vec2::ZERO;
    for event in mouse_motion_events.read() {
        cursor_delta += event.delta;
    }

    control_events.send(ControlAction::Look(mouse_rotate_sensitivity * cursor_delta));
}

/// Sends [`ControlAction`] events based on keyboard input.
fn keyboard_input(
    mut movement_event_writer: EventWriter<ControlAction>,
    keyboard_input: Res<Input<KeyCode>>,
    controllers: Query<&CharacterController>,
) {
    // let up = keyboard_input.any_pressed([KeyCode::W, KeyCode::Up]);
    // let down = keyboard_input.any_pressed([KeyCode::S, KeyCode::Down]);
    // let left = keyboard_input.any_pressed([KeyCode::A, KeyCode::Left]);
    // let right = keyboard_input.any_pressed([KeyCode::D, KeyCode::Right]);

    // let horizontal = right as i8 - left as i8;
    // let vertical = up as i8 - down as i8;
    // let direction = Vector2::new(horizontal as Scalar, vertical as Scalar).clamp_length_max(1.0);

    // Can only control one camera at a time.
    let controller = if let Some(controller) = controllers.iter().find(|c| c.enabled) {
        controller
    } else {
        return;
    };
    let CharacterController {
        translate_sensitivity,
        ..
    } = *controller;

    for (key, dir) in [
        (KeyCode::W, Vec3::Z),
        (KeyCode::A, Vec3::X),
        (KeyCode::S, -Vec3::Z),
        (KeyCode::D, -Vec3::X),
        (KeyCode::ShiftLeft, -Vec3::Y),
        (KeyCode::Space, Vec3::Y),
    ]
    .iter()
    .cloned()
    {
        if keyboard_input.pressed(key) {
            movement_event_writer.send(ControlAction::Move(translate_sensitivity * dir));
        }
    }
}

/// Updates the [`Grounded`] status for character controllers.
fn update_grounded(
    mut commands: Commands,
    mut query: Query<
        (Entity, &ShapeHits, &Rotation, Option<&MaxSlopeAngle>),
        With<CharacterController>,
    >,
) {
    for (entity, hits, rotation, max_slope_angle) in &mut query {
        // The character is grounded if the shape caster has a hit with a normal
        // that isn't too steep.
        let is_grounded = hits.iter().any(|hit| {
            if let Some(angle) = max_slope_angle {
                rotation.rotate(-hit.normal2).angle_between(Vector::Y).abs() <= angle.0
            } else {
                true
            }
        });

        if is_grounded {
            commands.entity(entity).insert(Grounded);
        } else {
            commands.entity(entity).remove::<Grounded>();
        }
    }
}

/// Responds to [`ControlAction`] events and moves character controllers accordingly.
fn movement(
    time: Res<Time>,
    mut control_event_reader: EventReader<ControlAction>,
    mut cameras: Query<(
        &CharacterController,
        &mut LookTransform,
        &MovementAcceleration,
        &JumpImpulse,
        &mut LinearVelocity,
        Has<Grounded>,
    )>,
) {
    // Can only control one camera at a time.
    if let Some((
        _,
        mut transform,
        movement_acceleration,
        jump_impulse,
        mut linear_velocity,
        is_grounded,
    )) = cameras.iter_mut().find(|c| c.0.enabled)
    {
        // Get inner angles for moving camera
        let look_vector = transform.look_direction().unwrap();
        let mut look_angles = LookAngles::from_vector(look_vector);

        let yaw_rot = Quat::from_axis_angle(Vec3::Y, look_angles.get_yaw());
        let rot_x = yaw_rot * Vec3::X;
        let rot_y = yaw_rot * Vec3::Y;
        let rot_z = yaw_rot * Vec3::Z;

        // Precision is adjusted so that the example works with
        // both the `f32` and `f64` features. Otherwise you don't need this.
        let dt = time.delta_seconds_f64().adjust_precision();

        for event in control_event_reader.read() {
            match event {
                ControlAction::Look(delta) => {
                    // Rotates with pitch and yaw.
                    look_angles.add_yaw(dt * -delta.x);
                    look_angles.add_pitch(dt * -delta.y);
                }
                ControlAction::Move(delta) => {
                    // Translates up/down (Y) left/right (X) and forward/back (Z).
                    transform.eye +=
                        dt * delta.x * rot_x + dt * delta.y * rot_y + dt * delta.z * rot_z;

                    // Updates linear velocity with location of eye and acceleration
                    linear_velocity.x += movement_acceleration.0 * transform.eye.x;
                    linear_velocity.z -= movement_acceleration.0 * transform.eye.z;

                    if is_grounded {
                        linear_velocity.y = jump_impulse.0 * transform.eye.y;
                    }
                }
            }
        }
    }
}

/// Slows down movement in the XZ plane.
fn apply_movement_damping(mut query: Query<(&MovementDampingFactor, &mut LinearVelocity)>) {
    for (damping_factor, mut linear_velocity) in &mut query {
        // We could use `LinearDamping`, but we don't want to dampen movement along the Y axis
        linear_velocity.x *= damping_factor.0;
        linear_velocity.z *= damping_factor.0;
    }
}
