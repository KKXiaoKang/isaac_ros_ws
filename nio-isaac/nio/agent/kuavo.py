from copy import deepcopy
from typing import Dict, List, Tuple, Union

import torch
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.prims import RigidPrim
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationActions

from .kuavo_cfg import KuavoCfg


class Kuavo(Articulation):
    def __init__(self, usd_path):
        self.cfg = deepcopy(KuavoCfg)
        self._prim_path = "/World/Kuavo"
        self._arm_idx = []
        self._leg_idx = []
        self._default_dof_effort_limit = None
        self._default_dof_stiffness = None
        self._default_dof_damping = None
        self._default_dof_pos = None
        self._default_dof_vel = None
        self._default_dof_effort = None

        add_reference_to_stage(usd_path=usd_path, prim_path=self._prim_path)

        super().__init__(
            prim_path=self._prim_path,
            name="Kuavo",
            position=torch.zeros(3),
            orientation=torch.tensor([1, 0, 0, 0]),
        )
        self._bodies = {}

    def initialize(self, physics_sim_view=None):
        self._arm_idx = []
        self._leg_idx = []
        super().initialize(physics_sim_view)
        for body_name in self._articulation_view.body_names:
            body = RigidPrim(prim_path=f"{self._prim_path}/{body_name}", name=body_name)

            body.initialize()
            body._rigid_prim_view.enable_gravities()  # pylint: disable=W0212
            self._bodies[body_name] = body

        # -- meta-information
        self._process_info_cfg()
        # -- set default gains
        self.set_gains(kps=self._default_dof_stiffness, kds=self._default_dof_damping)
        # -- set default state
        self.set_joints_default_state(
            positions=self._default_dof_pos,
            velocities=self._default_dof_vel,
            efforts=self._default_dof_effort,
        )
        self.set_linear_velocity(torch.zeros(3))
        self.set_angular_velocity(torch.zeros(3))
        self.post_reset()

    def get_bodies(self):
        return self._bodies

    def get_joint_names(self):
        return self._articulation_view.dof_names

    def get_arm_index(self):
        return self._arm_idx

    def get_obs(self):
        return {
            "joint_state": self.get_joint_state(),
            "body_state": self.get_root_state(),
            "stiffness": self.get_gains()[0],
            "dampings": self.get_gains()[1],
        }

    def get_joint_state(self) -> Dict:
        joint_positions = self.get_joint_positions()
        joint_velocities = self.get_joint_velocities()
        joint_applied_efforts = self.get_applied_joint_efforts()
        arms_positions = joint_positions[self._arm_idx]
        arms_velocities = joint_velocities[self._arm_idx]
        arms_applied_effort = joint_applied_efforts[self._arm_idx]
        legs_positions = joint_positions[self._leg_idx]
        legs_velocities = joint_velocities[self._leg_idx]
        legs_applied_effort = joint_applied_efforts[self._leg_idx]

        return {
            "arms_positions": arms_positions,
            "arms_velocities": arms_velocities,
            "arms_applied_effort": arms_applied_effort,
            "legs_positions": legs_positions,
            "legs_velocities": legs_velocities,
            "legs_applied_effort": legs_applied_effort,
        }

    def get_root_state(self) -> Dict:
        return {
            "world_pos": self.get_world_pose()[0],
            "world_orient": self.get_world_pose()[1],
            "linear_velocities": self.get_linear_velocity(),
            "angular_velocities": self.get_angular_velocity(),
        }

    def get_joint_index(self, joint_name: str) -> int:
        """Get a DOF index in the joint buffers given its name

        Args:
            dof_name (str): name of the joint that corresponds to the degree of
            freedom to query

        Returns:
            int: index of the degree of freedom in the joint buffers

        Example:

        ``` python title=""
        >>> # get the index of the left finger joint: panda_finger_joint1
        >>> agent.get_joint_index("panda_finger_joint1")
        7
        ```
        """
        return self._articulation_view.get_dof_index(joint_name)

    @property
    def articulatiuon_controller(self):
        """Get the articulation controller.

        Returns:
            ArticulationController: A Proportional-Derivative controller that can apply
            position targets,  velocity targets and efforts
        """

        return super().get_articulation_controller()

    def set_gains(
        self,
        kps=None,
        kds=None,
        joint_indices: List[int] = None,
    ) -> None:
        if kps is not None and not isinstance(kps, torch.Tensor):
            raise TypeError("kps must be a torch.Tensor or None")
        if kds is not None and not isinstance(kds, torch.Tensor):
            raise TypeError("kds must be a torch.Tensor or None")

        if joint_indices is None:
            self.articulatiuon_controller.set_gains(kps=kps, kds=kds)
            return
        self._articulation_view.set_gains(
            kps=kps, kds=kds, joint_indices=torch.tensor(joint_indices)
        )

    def get_arm_idx(self) -> List[int]:
        return self._arm_idx

    def get_leg_idx(self) -> List[int]:
        return self._leg_idx

    def get_gains(self, joint_indices=None) -> Tuple[torch.Tensor, torch.Tensor]:
        """Get the proportional gains (stiffness) and derivative gains (damping) for
        the agent's joints.

        Args:
            joint_indices (Optional[List[int]]): The indices of the joints to get the
                gains for. If None, the gains for all joints will be returned.
                Defaults to None.

        Returns:
            Tuple[torch.Tensor, torch.Tensor]: A tuple containing the proportional
            gains (stiffness) and derivative gains (damping).

        Example:

        ``` python title=""
        >>> # Get gains for all joints
        >>> kps, kds = agent.get_gains()
        >>> # Get gains for joints [1, 2, 3] only
        >>> kps, kds = agent.get_gains(joint_indices=[1, 2, 3])
        ```
        """
        kps, kds = self.articulatiuon_controller.get_gains()
        if joint_indices is None:
            return kps, kds
        return kps[joint_indices], kds[joint_indices]

    def apply_action(self, command_dict: Dict[str, Union[str, List, torch.Tensor]]):
        if command_dict is None:
            return
        if command_dict.get("arms", None):
            self._pre_physics_step(self._arm_idx, command_dict["arms"])
        if command_dict.get("legs", None):
            self._pre_physics_step(self._leg_idx, command_dict["legs"])

    @property
    def physics_view(self):
        """Get the physics simulation view.

        Returns:
            omni.physics.tensors.SimulationView: The physics simulation view.
        """
        # pylint:disable = W0212
        return self._articulation_view._physics_view

    # ================================ Internal helper==================================
    def _process_info_cfg(self) -> None:
        """Post processing of configuration parameters."""
        # -- meta_info: joint index
        joint_names = self.dof_names
        self._default_dof_effort_limit = torch.zeros(self.num_dof)
        self._default_dof_stiffness = torch.zeros(self.num_dof)
        self._default_dof_damping = torch.zeros(self.num_dof)
        self._default_dof_pos = torch.zeros(self.num_dof)
        self._default_dof_vel = torch.zeros(self.num_dof)
        self._default_dof_effort = torch.zeros(self.num_dof)

        default_state = self.cfg["default_state"]
        for idx, joint_name in enumerate(joint_names):
            if default_state.get(joint_name, None) is not None:
                self._default_dof_effort_limit[idx] = default_state[
                    joint_name
                ].effort_limit
                self._default_dof_stiffness[idx] = default_state[joint_name].stiffness
                self._default_dof_damping[idx] = default_state[joint_name].damping
                self._default_dof_pos[idx] = default_state[joint_name].dof_pos
                self._default_dof_vel[idx] = default_state[joint_name].dof_vel
                self._default_dof_effort[idx] = default_state[joint_name].dof_effort
                if joint_name in self.cfg["arm_names"]:
                    self._arm_idx.append(idx)
                if joint_name in self.cfg["leg_names"]:
                    self._leg_idx.append(idx)

    def _pre_physics_step(self, idx: List, command_dict: dict = None):
        if not command_dict:
            return

        if command_dict.get("stiffness", None) or command_dict.get("dampings", None):
            # Update Gains:
            self.set_gains(
                kps=command_dict.get("stiffness", None),
                kds=command_dict.get("dampings", None),
                joint_indices=idx,
            )

        # Set target action
        joint_value = command_dict.get("joint_values", None)
        target_action = None
        if command_dict.get("ctrl_mode", None) == "position":
            target_action = ArticulationActions(
                joint_positions=joint_value,
                joint_indices=idx,
            )
            self._articulation_view.apply_action(target_action)
        if command_dict.get("ctrl_mode", None) == "velocity":
            target_action = ArticulationActions(
                joint_velocities=joint_value,
                joint_indices=idx,
            )
            # print(f" index : {idx} ----------------- leg --- velocity ------ control ---------------------")
            self._articulation_view.apply_action(target_action)

        if command_dict.get("ctrl_mode", None) == "effort":
            target_action = ArticulationActions(
                joint_efforts=joint_value,
                joint_indices=idx,
            )
            # print(f" index : {idx} ----------------- leg --- effort ------ control ---------------------")
            self._articulation_view.apply_action(target_action)

        return

    def get_total_mass(self, verbose=False):
        """
        获取机器人所有刚体的总质量
        Args:
            verbose (bool): 是否打印详细信息
        Returns:
            float: 总质量(kg)
        """
        total_mass = 0.0
        for body_name, body in self._bodies.items():
            mass = body._rigid_prim_view.get_masses()[0]
            if verbose:
                print(f"Body: {body_name}, Mass: {mass:.3f} kg")
            total_mass += mass
        if verbose:
            print(f"Total mass: {total_mass:.3f} kg")
        return total_mass
