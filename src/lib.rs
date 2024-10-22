//! [FABRIK] inverse kinematics implementation for [Bevy].
//! 
//! [FABRIK]: https://www.andreasaristidou.com/FABRIK.html
//! [Bevy]: https://bevyengine.org

mod constraint;
mod chain;
mod util;

use bevy::prelude::*;

pub use crate::chain::IkChain;
pub use crate::constraint::{SwingConstraint, TwistConstraint};

fn ik_solve(
    mut chain_query: Query<(Entity, &mut IkChain)>,
    mut constraint_query: Query<(Option<&SwingConstraint>, Option<&TwistConstraint>)>,
    mut transform_query: Query<(Option<&Parent>, &mut Transform, &GlobalTransform)>,
) {
    // TODO: Multi-chain structures
    for (entity, chain) in chain_query.iter_mut() {
        if let Err(e) = chain.solve(&entity, &mut constraint_query, &mut transform_query) {
            warn!("Failed to solve IK chain: {:?}", e);
        }
    }
}

/// [`Plugin`] for inverse kinematics. Needs to be added to the [`App`] for
/// [`IkChain`] components to solve and apply transforms.
pub struct InverseKinematicsPlugin;

impl Plugin for InverseKinematicsPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<IkChain>()
            .register_type::<SwingConstraint>()
            .register_type::<TwistConstraint>()
            .add_systems(Update, (
                ik_solve,
            ));
    }
}
