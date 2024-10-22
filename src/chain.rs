use bevy::{
    ecs::{
        component::{ComponentHooks, ComponentId, StorageType},
        query::QueryEntityError,
        system::QueryLens,
        world::DeferredWorld
    },
    math::{
        Affine3A,
        Vec3A
    },
    prelude::*
};

use crate::constraint::*;

const DEFAULT_TARGET_SQ_THRESHOLD: f32 = 0.001;
const DEFAULT_MAX_ITTERATIONS: usize = 10;

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
    ) -> Result<(), QueryEntityError> {
        if !self.enabled || transform_query.get(*entity)?.2.translation().distance_squared(self.target) < self.threshold {
            return Ok(());
        }

        let mut constraint_lens : QueryLens<(
            Option<&Parent>,
            &GlobalTransform,
            Option<&SwingConstraint>,
            Option<&TwistConstraint>
        )> = constraint_query.join_filtered(transform_query);
        let lens_query = constraint_lens.query();

        // Collect joint transforms and constraints, if any
        let mut joint_transforms : Vec<(GlobalTransform, Vec<&dyn Constraint>)> = vec![];
        let mut current_joint = *entity;
        for i in 0..self.joint_count {
            let (
                parent,
                global_transform,
                swing_constraint,
                twist_constraint
            ) = lens_query.get(current_joint)?;

            let constraints = vec![
                swing_constraint.map(|c| c as &dyn Constraint),
                twist_constraint.map(|c| c as &dyn Constraint),
            ].into_iter().flatten().collect();

            joint_transforms.push((*global_transform, constraints));

            // We don't need the root joint's parent
            if i < self.joint_count - 1 {
                current_joint = **parent.unwrap();
            }
        }

        // Solve
        for _ in 0..self.max_iterations {
            if self.target.distance_squared(joint_transforms.first().unwrap().0.translation()) < self.threshold {
                break;
            }

            self.backward_pass(&mut joint_transforms);
            self.forward_pass(&mut joint_transforms);
        }

        // Apply the transforms
        let mut current_joint = *entity;
        for i in 0..self.joint_count - 1 {
            let (parent, mut transform, _) = transform_query.get_mut(current_joint)?;
            let updated_transform = joint_transforms[i].0.reparented_to(&joint_transforms[i + 1].0);
            transform.translation = updated_transform.translation;
            transform.rotation = updated_transform.rotation;
            current_joint = **parent.unwrap();
        }

        Ok(())
    }

    fn backward_pass(
        &self,
        joints: &mut Vec<(GlobalTransform, Vec<&dyn Constraint>)>,
    ) {
        let origin = joints.last().unwrap().0.translation();

        joints[0].0 = Affine3A {
            matrix3: joints[0].0.affine().matrix3,
            translation: Vec3A::from(self.target),
        }.into();

        for i in 1..joints.len() {
            let direction: Vec3A = (joints[i].0.translation() - joints[i - 1].0.translation()).normalize().into();

            joints[i].0 = Affine3A {
                matrix3: joints[i - 1].0.affine().matrix3,
                translation: joints[i - 1].0.affine().translation + direction * self.joint_lengths[i - 1],
            }.into();
        }
        
        // Set root transform back to origin
        joints.last_mut().unwrap().0 = Affine3A {
            matrix3: joints.last().unwrap().0.affine().matrix3,
            translation: origin.into(),
        }.into();
    }

    fn forward_pass(
        &self,
        joints: &mut Vec<(GlobalTransform, Vec<&dyn Constraint>)>,
    ) {
        let root_end_index = joints.len() - 2;

        joints[root_end_index].0 = Affine3A {
            matrix3: joints[root_end_index].0.affine().matrix3,
            translation: joints.last().unwrap().0.affine().translation + Vec3A::from(self.root_normal * self.joint_lengths[root_end_index]),
        }.into();

        for i in (0..root_end_index).rev() {
            let child_transform = joints[i + 1].0;
            let parent_transform = joints[i].0;

            let direction = (parent_transform.translation() - child_transform.translation()).normalize();

            joints[i + 1].0 = rotate_and_constrain(direction, self.root_normal.into(), child_transform, joints[i + 1].1.clone());

            joints[i].0 = Affine3A {
                matrix3: parent_transform.affine().matrix3,
                translation: joints[i + 1].0.affine().translation + Vec3A::from(joints[i + 1].0.up() * self.joint_lengths[i + 1]),
            }.into();
        }
    }
}

fn ik_chain_insert_hook(
    mut world: DeferredWorld,
    entity: Entity,
    _component_id: ComponentId,
) {
    let joint_count = {world.get::<IkChain>(entity).unwrap().joint_count};

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
                panic!("Parent not found for entity {:?} while building IK chain (length {:?})", tail, joint_count);
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
                panic!("Transform not found for entity {:?} while building IK chain (length {:?})", tail, joint_count);
            };
            transform.translation()
        };

        joint_normal = tail_pos - joint_pos;
        lengths.push(joint_normal.length());
        tail = joint;
    }
    let Some(mut chain) = world.get_mut::<IkChain>(entity) else {
        panic!("IkChain component not found for entity {:?} while building IK chain.", entity);
    };

    chain.joint_lengths = lengths;
    chain.root_normal = Dir3::new_unchecked(joint_normal.normalize_or(Vec3::Y));
}