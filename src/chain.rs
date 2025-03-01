use std::f32::consts::PI;

use bevy::{
    ecs::{
        component::{ComponentHooks, ComponentId, StorageType},
        query::QueryEntityError,
        system::QueryLens,
        world::DeferredWorld,
    },
    math::{Affine3A, Vec3A},
    prelude::*,
};

use crate::constraint::*;

const DEFAULT_TARGET_SQ_THRESHOLD: f32 = 0.001;
const DEFAULT_MAX_ITTERATIONS: usize = 10;

/// Wrapper for the [QueryEntityError] specifically for IK chain errors.
/// Used to work around the [QueryEntityError] lifetime introduced in Bevy 0.15.
#[derive(Clone, Copy)]
pub enum IkChainError {
    IkComponentQueryError,
}

impl core::error::Error for IkChainError {}

impl core::fmt::Display for IkChainError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Failed to query required IK chain components")
    }
}

impl core::fmt::Debug for IkChainError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "IkChainError")
    }
}

impl From<QueryEntityError<'_>> for IkChainError {
    fn from(_: QueryEntityError) -> Self {
        Self::IkComponentQueryError
    }
}

/// Main component that defines an IK chain. Add this to an end entity
/// of a chain of entities to solve the chain's inverse kinematics.
#[derive(Reflect)]
pub struct IkChain {
    /// If false, the chain will not be solved.
    pub enabled: bool,
    /// The target position for the chain to reach.
    pub target: Vec3,
    /// The squared distance threshold for the chain to stop solving.
    pub threshold: f32,
    /// The maximum number of iterations to solve the chain.
    pub max_iterations: usize,
    pub(crate) joint_count: usize,
    pub(crate) joint_lengths: Vec<f32>,
    root_normal: Dir3,
}

impl Component for IkChain {
    const STORAGE_TYPE: StorageType = StorageType::Table;

    fn register_component_hooks(hooks: &mut ComponentHooks) {
        hooks.on_insert(ik_chain_insert_hook);
    }
}

impl IkChain {
    /// Creates a new IK chain with the given number of joints.
    ///
    /// The parent entities need to already exist in the [`World`] and
    /// have [`GlobalTransform`] components correctly set up.
    pub fn new(joint_count: usize) -> Self {
        Self {
            enabled: true,
            target: Vec3::ZERO,
            threshold: DEFAULT_TARGET_SQ_THRESHOLD,
            max_iterations: DEFAULT_MAX_ITTERATIONS,
            joint_count,
            joint_lengths: vec![],
            root_normal: Dir3::Y,
        }
    }

    pub(crate) fn solve(
        &self,
        entity: &Entity,
        constraint_query: &mut Query<(Option<&SwingConstraint>, Option<&TwistConstraint>)>,
        transform_query: &mut Query<(Option<&Parent>, &mut Transform, &GlobalTransform)>,
    ) -> Result<(), IkChainError> {
        if !self.enabled {
            return Ok(());
        }

        let dist = transform_query
            .get(*entity)
            .map(|(_, _, gt)| gt.translation().distance_squared(self.target))?;

        if dist < self.threshold {
            return Ok(());
        }

        let mut constraint_lens: QueryLens<(
            Option<&Parent>,
            &GlobalTransform,
            Option<&SwingConstraint>,
            Option<&TwistConstraint>,
        )> = constraint_query.join_filtered(transform_query);
        let lens_query = constraint_lens.query();

        // Collect joint transforms and constraints, if any
        let mut joint_transforms: Vec<(GlobalTransform, Vec<&dyn Constraint>)> = vec![];
        let mut current_joint = *entity;
        for i in 0..self.joint_count {
            let (parent, global_transform, swing_constraint, twist_constraint) =
                lens_query.get(current_joint)?;

            let constraints = [
                swing_constraint.map(|c| c as &dyn Constraint),
                twist_constraint.map(|c| c as &dyn Constraint),
            ]
            .into_iter()
            .flatten()
            .collect();

            joint_transforms.push((*global_transform, constraints));

            // We don't need the root joint's parent
            if i < self.joint_count - 1 {
                current_joint = **parent.unwrap();
            }
        }

        // Solve
        for _ in 0..self.max_iterations {
            if self
                .target
                .distance_squared(joint_transforms.first().unwrap().0.translation())
                < self.threshold
            {
                break;
            }

            self.backward_pass(&mut joint_transforms[..]);
            self.forward_pass(&mut joint_transforms[..]);
        }

        // Apply the transforms
        let mut current_joint = *entity;
        for i in 0..self.joint_count - 1 {
            let (parent, mut transform, _) = transform_query.get_mut(current_joint)?;
            let updated_transform = joint_transforms[i]
                .0
                .reparented_to(&joint_transforms[i + 1].0);
            transform.translation = updated_transform.translation;
            transform.rotation = updated_transform.rotation;
            current_joint = **parent.unwrap();
        }

        Ok(())
    }

    fn backward_pass(&self, joints: &mut [(GlobalTransform, Vec<&dyn Constraint>)]) {
        let origin = joints.last().unwrap().0.translation();

        joints[0].0 = Affine3A {
            matrix3: joints[0].0.affine().matrix3,
            translation: Vec3A::from(self.target),
        }
        .into();

        for i in 1..joints.len() {


            let direction: Vec3A = (joints[i].0.translation() - joints[i - 1].0.translation())
                .normalize()
                .into();

            joints[i].0 = Affine3A {
                matrix3: joints[i - 1].0.affine().matrix3,
                translation: joints[i - 1].0.affine().translation
                    + direction * self.joint_lengths[i - 1],
            }
            .into();
        }

        // Set root transform back to origin
        joints.last_mut().unwrap().0 = Affine3A {
            matrix3: joints.last().unwrap().0.affine().matrix3,
            translation: origin.into(),
        }
        .into();
    }

    fn forward_pass(&self, joints: &mut [(GlobalTransform, Vec<&dyn Constraint>)]) {
        let root_end_index = joints.len() - 2;

        joints[root_end_index].0 = Affine3A {
            matrix3: joints[root_end_index].0.affine().matrix3,
            translation: joints.last().unwrap().0.affine().translation
                + Vec3A::from(self.root_normal * self.joint_lengths[root_end_index]),
        }
        .into();


        let mut root_angle = 0.0;
        for i in (0..root_end_index).rev() {
            let a_transform = joints[i + 1].0; // pervious i
            let b_transform = joints[i].0; // new in loop

            let a = a_transform.translation().xy();
            let b = b_transform.translation().xy();

            let diff_angle_raw = wrapf(angle_between_points(a, b) - root_angle, -PI, PI);

            
            let mut min = -PI;
            let mut max = PI;

            for test  in  joints[i+1].1.iter() {
                min = -test.get_angle();
                max = test.get_angle();
            }
            

            let diff_angle = diff_angle_raw.clamp(min, max);
            let angle = root_angle + diff_angle;

            let rotated_vector = rotate_vector(Vec2::new(self.joint_lengths[i+1], 0.0), angle);


            joints[i+1].0 = joints[i+1].0.compute_transform().with_rotation(Quat::from_rotation_z(angle + PI/2.0)).into();

            joints[i].0 = Affine3A {
                matrix3: b_transform.affine().matrix3,
                translation: Vec3A::new(a.x, a.y, 0.0)
                    + Vec3A::from( rotated_vector.extend(0.0)),
            }
            .into();

            root_angle = angle;

        }
    }
}


fn angle_between_points(p1: Vec2, p2: Vec2) -> f32 {
    let delta_y = p2.y - p1.y;
    let delta_x = p2.x - p1.x;
    
    // Use atan2 to handle quadrant correctly
    let angle_radians =  f32::atan2(delta_y, delta_x);
    
    return angle_radians; 
}

#[inline]
pub fn wrapf(mut n: f32, mut min_limit: f32, mut max_limit: f32) -> f32 {
    if n >= min_limit && n <= max_limit {
        return n;
    }

    if max_limit == 0.0 && min_limit == 0.0 {
        return 0.0;
    }

    max_limit -= min_limit;

    let offset = min_limit;
    min_limit = 0.0;
    n -= offset;

    let num_of_max = (n / max_limit).abs().floor();

    if n >= max_limit {
        n -= num_of_max * max_limit;
    } else if n < min_limit {
        n += (num_of_max + 1.0) * max_limit;
    }

    n + offset
}


fn rotate_vector(vector: Vec2, angle: f32) -> Vec2 {


    let cos_angle =  f32::cos(angle);

    let sin_angle = f32::sin(angle);

    

    Vec2::new(cos_angle * vector.x - sin_angle * vector.y, sin_angle * vector.x + cos_angle * vector.y)

}

fn ik_chain_insert_hook(mut world: DeferredWorld, entity: Entity, _component_id: ComponentId) {
    let joint_count = { world.get::<IkChain>(entity).unwrap().joint_count };

    if joint_count < 2 {
        panic!("IK chain must have at least 2 joints.");
    }

    // We'll use the last value later as root normal
    let mut joint_normal = Vec3::ZERO;
    let mut tail = entity;
    let mut lengths = Vec::<f32>::with_capacity(joint_count - 1);

    for _ in 1..joint_count {
        let joint = {
            let Some(joint) = world.get::<Parent>(tail) else {
                panic!(
                    "Parent not found for entity {:?} while building IK chain (length {:?})",
                    tail, joint_count
                );
            };
            **joint
        };

        let joint_pos = {
            let Some(transform) = world.get::<GlobalTransform>(joint) else {
                panic!("Parent transform not found for entity {:?} while building IK chain (length {:?})", joint, joint_count);
            };
            transform.translation()
        };

        let tail_pos = {
            let Some(transform) = world.get::<GlobalTransform>(tail) else {
                panic!(
                    "Transform not found for entity {:?} while building IK chain (length {:?})",
                    tail, joint_count
                );
            };
            transform.translation()
        };

        joint_normal = tail_pos - joint_pos;
        lengths.push(joint_normal.length());
        tail = joint;
    }
    let Some(mut chain) = world.get_mut::<IkChain>(entity) else {
        panic!(
            "IkChain component not found for entity {:?} while building IK chain.",
            entity
        );
    };

    chain.joint_lengths = lengths;
    
    let result = Dir3::new(joint_normal.normalize_or(Vec3::Y));
    if let Ok(re) = result {
        chain.root_normal = re
    } else {
        chain.root_normal = Dir3::new_unchecked(Vec3::Y);
    }
    

    info!("{:?}", chain.root_normal);
}
