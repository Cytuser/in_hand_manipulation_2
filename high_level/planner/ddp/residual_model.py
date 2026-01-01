import numpy as np
import crocoddyl
import pinocchio as pin

from common.common_ddp import convert_rpy_to_matrix, convert_quat_to_matrix, get_angvel_over_euler_derivative, CalcNQdot2W


class ResidualModelPairCollision(crocoddyl.ResidualModelAbstract):
    """
        Note that pin_model and geom_model only include the hand.
        While state include both the hand and the object.
        Thus state_slice is the hand dofs in state.
    """
    def __init__(self, state, nu, pin_model, geom_model, pair_indices, state_slice):
        # TODO: IMPORTANT! According to 
        # https://github.com/loco-3d/crocoddyl/blob/da92f67394c07c987458a8cb24bc0e33edd64227/include/crocoddyl/core/residual-base.hxx#L88-L105,
        # setting v_dependent to False for quasi-static model (nv=0) will leave residual.Lx=0,
        # which is incorrect. So we set v_dependent to True.

        # TODO: IMPORTANT! Please set joint_index to parentJoint of geom1,
        # as geom2 is considered as the (fixed) environment.
        crocoddyl.ResidualModelAbstract.__init__(self, state, 3, nu, True, True, False)

        joint_ids = []
        for pair_index in pair_indices:
            assert len(geom_model.collisionPairs) > pair_index
            geom_id1 = geom_model.collisionPairs[pair_index].first
            geom_id2 = geom_model.collisionPairs[pair_index].second
            parent_joint_id1 = geom_model.geometryObjects[geom_id1].parentJoint
            parent_joint_id2 = geom_model.geometryObjects[geom_id2].parentJoint
            joint_ids.append((parent_joint_id1, parent_joint_id2))

        self.num_pairs = len(pair_indices)

        self.pin_model = pin_model
        self.pin_data = self.pin_model.createData()
        self.geom_model = geom_model
        self.geom_data = self.geom_model.createData()

        self.pair_ids = pair_indices
        self.joint_ids = joint_ids
        self.state_slice = state_slice

        self.q = None

    def calc(self, data, x, u):
        q = x[self.state_slice]
        self.q = q
        pin.updateGeometryPlacements(self.pin_model, self.pin_data, self.geom_model, self.geom_data, q)
        
        for i in range(self.num_pairs):
            pair_id = self.pair_ids[i]
            pin.computeDistance(self.geom_model, self.geom_data, pair_id)
            data.r[:] += self.geom_data.distanceResults[pair_id].getNearestPoint1() - \
                            self.geom_data.distanceResults[pair_id].getNearestPoint2()

    def calcDiff(self, data, x, u):
        q = self.q

        pin.computeJointJacobians(self.pin_model, self.pin_data, q)
        for i in range(self.num_pairs):
            pair_id = self.pair_ids[i]
            joint_id1, joint_id2 = self.joint_ids[i]
            
            d1 = self.geom_data.distanceResults[pair_id].getNearestPoint1() - \
                        self.pin_data.oMi[joint_id1].translation
            J1 = pin.getJointJacobian(self.pin_model, self.pin_data, joint_id1, pin.LOCAL_WORLD_ALIGNED)

            J1[:3] += np.matmul(pin.skew(d1).T, J1[-3:])

            d2 = self.geom_data.distanceResults[pair_id].getNearestPoint2() - \
                        self.pin_data.oMi[joint_id2].translation
            J2 = pin.getJointJacobian(self.pin_model, self.pin_data, joint_id2, pin.LOCAL_WORLD_ALIGNED)

            J2[:3] += np.matmul(pin.skew(d2).T, J2[-3:])

            data.Rx[:3, self.state_slice] += (J1[:3] - J2[:3])


class ResidualModelFrameRotation(crocoddyl.ResidualModelAbstract):
    """
        Residual model for relative frame rotation.
        Reference: crocoddyl GitHub repo (https://github.com/loco-3d/crocoddyl/blob/9919619930878f6c4c015cdf94dc7346c986580a/include/crocoddyl/multibody/residuals/frame-rotation.hxx)
        :param Rref: reference frame rotation (3x3 matrix)
        :param xrot_slc: slice of frame rotation in state (default to euler xyz)
    """
    def __init__(self, state, nu, Rref, xrot_slc):
        crocoddyl.ResidualModelAbstract.__init__(self, state, 3, nu, True, True, False)

        self.Rref_ = Rref.copy()
        self.oRf_inv_ = Rref.copy().T

        if isinstance(xrot_slc, slice):
            if xrot_slc.stop - xrot_slc.start == 3:
                self.xrot_type_ = 'RPY'
            elif xrot_slc.stop - xrot_slc.start == 4:
                self.xrot_type_ = 'Quat'
            else:
                raise ValueError(f"Invalid xrot_slc {xrot_slc}")
        else:
            assert isinstance(xrot_slc, list)
            if len(xrot_slc) == 3:
                self.xrot_type_ = 'RPY'
            elif len(xrot_slc) == 4:
                self.xrot_type_ = 'Quat'
            else:
                raise ValueError(f"Invalid xrot_slc {xrot_slc}")
        
        self.xrot_slc_ = xrot_slc

    def set_reference(self, rotation):
        self.Rref_ = rotation.copy()
        self.oRf_inv_ = rotation.copy().T

    def calc(self, data, x, u):
        if self.xrot_type_ == 'RPY':
            rpy = x[self.xrot_slc_]
            oMf_rot = convert_rpy_to_matrix(rpy)
        elif self.xrot_type_ == 'Quat':
            quat = x[self.xrot_slc_]
            oMf_rot = convert_quat_to_matrix(quat)

        # data.rRf[:] = self.oRf_inv_ @ oMf_rot
        self.rRf = self.oRf_inv_ @ oMf_rot
        data.r[:] = pin.log3(self.rRf)

    def calcDiff(self, data, x, u):
        rJf = pin.Jlog3(self.rRf)

        if self.xrot_type_ == 'RPY':
            rpy = x[self.xrot_slc_]
            fJf = get_angvel_over_euler_derivative(rpy, seq='RPY')
        elif self.xrot_type_ == 'Quat':
            quat = x[self.xrot_slc_]
            fJf = CalcNQdot2W(quat)

        data.Rx[:, self.xrot_slc_] = rJf @ fJf

class ResidualModelManipulabilityTCI(crocoddyl.ResidualModelAbstract):
    """
    Fixed logic: 
    1. Supports dynamic axis calculation by extracting object pose from x.
    2. Implements proper Grasp Matrix logic.
    """

    def __init__(
        self,
        state,
        nu: int,
        pin_model,
        tip_frame_ids: list,
        sphere_center: np.ndarray,
        hand_joint_slice,           # Slice for hand joints
        object_pose_slice=None,     # [NEW] Slice for object pose (7 dim: pos+quat)
        task_axis: np.ndarray = None,
        epsilon: float = 1e-4
    ):
        crocoddyl.ResidualModelAbstract.__init__(self, state, 1, nu, True, True, True)
        
        # 1. Dependency Injection
        self.pin_model = pin_model
        self.pin_data = pin_model.createData() # Thread safety
        
        self.tip_frame_ids = tip_frame_ids
        self.sphere_center = np.asarray(sphere_center).flatten()[:3]
        
        # 2. State Slicing
        self.hand_slice = hand_joint_slice
        self.object_slice = object_pose_slice # Needed for dynamic axis
        
        # 3. Task Configuration
        if task_axis is not None:
            self.task_axis = np.asarray(task_axis) / np.linalg.norm(task_axis)
            self.use_fixed_axis = True
        else:
            self.task_axis = np.array([0., 0., 1.])
            self.use_fixed_axis = False # Will use object_slice to compute axis
            
        self.target_rotation = None # Set via set_target_rotation
        self.epsilon = epsilon

    def set_target_rotation(self, target_quat):
        """Set target quaternion [x,y,z,w] for dynamic axis"""
        # Ensure input is pinocchio quaternion object
        if isinstance(target_quat, (np.ndarray, list)):
            # Assume input is [x,y,z,w] (Pinocchio convention)
            self.target_rotation = pin.Quaternion(
                target_quat[3], target_quat[0], target_quat[1], target_quat[2]
            )
        else:
            self.target_rotation = target_quat
        self.use_fixed_axis = False

    def calc(self, data, x, u):
        # 1. Update Hand Kinematics
        q_hand = x[self.hand_slice]
        pin.forwardKinematics(self.pin_model, self.pin_data, q_hand)
        pin.computeJointJacobians(self.pin_model, self.pin_data, q_hand)
        pin.updateFramePlacements(self.pin_model, self.pin_data)
        
        # 2. Determine Task Axis (The Object Twist Direction)
        if self.use_fixed_axis or self.object_slice is None or self.target_rotation is None:
            # Fallback to fixed axis
            rot_axis = self.task_axis
        else:
            # [FIXED] Dynamic Axis Logic
            # Extract object quaternion from state x
            # Assuming object state is [px, py, pz, x, y, z, w]
            obj_q_data = x[self.object_slice] 
            # Pinocchio quat is [x,y,z,w], usually at end of 7-vec
            curr_quat = pin.Quaternion(
                obj_q_data[6], obj_q_data[3], obj_q_data[4], obj_q_data[5]
            )
            
            # Compute Log(R_curr^T * R_target)
            rot_err = pin.log3(curr_quat.conjugate() * self.target_rotation)
            norm_err = np.linalg.norm(rot_err)
            
            if norm_err < 1e-4:
                # Already at target, gradient is zero
                data.r[0] = 0.0
                return
            
            rot_axis = rot_err / norm_err

        # 3. Compute TCI
        total_tci = 0.0
        
        for fid in self.tip_frame_ids:
            # A. Jacobian & Manipulability
            J = pin.getFrameJacobian(
                self.pin_model, self.pin_data, fid,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )[:3, :]
            M = J @ J.T
            
            # B. Transposed Grasp Matrix Mapping (v_des = G^T * V_obj)
            # V_obj = [0, w], so v_des = w x (p_tip - p_center)
            tip_pos = self.pin_data.oMf[fid].translation
            radius = tip_pos - self.sphere_center
            
            # [Math] This cross product is equivalent to G^T @ Twist
            v_des = np.cross(rot_axis, radius)
            
            norm_v = np.linalg.norm(v_des)
            if norm_v < 1e-6: continue
            
            u_tan = v_des / norm_v
            
            # C. Score
            total_tci += u_tan.T @ M @ u_tan

        # 4. Residual
        data.r[0] = 1.0 / (total_tci + self.epsilon)

    def calcDiff(self, data, x, u):
        pass # Using NumDiff

# ---------------------------------------------------------

def create_tci_cost_from_params(state, actuation, options):
    """Factory with NumDiff wrapper [FIXED]"""
    if options.W_MANIP <= 0 or options.pin_model_manip is None:
        return None
        
    # [FIXED] Pass object slice if available
    # Assuming standard state: [object(7) + hand(nq)]
    # You might need to adjust indices based on your exact state vector structure
    obj_slice = slice(0, 7) if options.sphere_joint_idx == 0 else None
    
    residual = ResidualModelManipulabilityTCI(
        state, actuation.nu,
        options.pin_model_manip,
        options.contact_frame_ids,
        options.sphere_center,
        options.hand_joint_slice,
        object_pose_slice=obj_slice, # Pass this!
        task_axis=None 
    )
    
    # Set target if available
    if options.target_rotation_manip is not None:
        residual.set_target_rotation(options.target_rotation_manip)

    # [FIXED] CRITICAL: Wrap with NumDiff!
    residual_diff = crocoddyl.ResidualModelNumDiff(residual)
    
    cost = crocoddyl.CostModelResidual(state, residual_diff)
    return cost