use bevy::prelude::*;

use crate::util::*;

// This will likely be useful more directly when Bevy supports trait queries.
pub(crate) trait Constraint {
    fn apply(&self, swing: Quat, twist: Quat) -> (Quat, Quat);
    fn get_angle(&self) -> f32;
}

/// Constrains the swing of a joint rotation by a given angle in radians.
#[derive(Component, Clone, Reflect)]
pub struct SwingConstraint(pub f32);

impl Constraint for SwingConstraint {
    fn apply(&self, swing: Quat, twist: Quat) -> (Quat, Quat) {
        (Quat::constrain(swing, self.0), twist)
    }

    fn get_angle(&self) -> f32 {

        self.0
    }
}

/// Constrains the twist of a joint rotation by a given angle in radians.
#[derive(Component, Clone, Reflect)]
pub struct TwistConstraint(pub f32);

impl Constraint for TwistConstraint {
    fn apply(&self, swing: Quat, twist: Quat) -> (Quat, Quat) {
        (swing, Quat::constrain(twist, self.0))
    }

    fn get_angle(&self) -> f32 {

        self.0
    }
}

pub(crate) fn rotate_and_constrain(
    direction: Vec3,
    normal: Vec3,
    transform: GlobalTransform,
    constraints: &[&dyn Constraint],
) -> GlobalTransform {
    let mut result = transform.compute_transform();

    let rotation_local = result.rotation.inverse() * Quat::from_rotation_arc(normal, direction);
    let (mut twist, mut swing) = rotation_local.decompose(Vec3::NEG_Z);

    for constraint in constraints {
        (swing, twist) = constraint.apply(swing, twist);
    }

    result.rotation = result.rotation * swing * twist;
    result.into()
}
