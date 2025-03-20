# import math
from dataclasses import dataclass

# -- Default joint positions
leg_initial_pos = [
    -0.01867,
    -0.00196,
    -0.65815,
    0.86691,
    -0.31346,
    0.01878,
    0.01868,
    0.00197,
    -0.65815,
    0.86692,
    -0.31347,
    -0.01878,
]

arm_initial_pos = [0] * 14
TAU_LEG_FLAG = True
TAU_ARM_FLAG = False

if TAU_LEG_FLAG: # 纯力矩控制模式
    leg_dof_stiffness = {
        "leg_l1_joint": 0,
        "leg_l2_joint": 0,
        "leg_l3_joint": 0,
        "leg_l4_joint": 0,
        "leg_l5_joint": 0, # ankle
        "leg_l6_joint": 0, # ankle
        "leg_r1_joint": 0,
        "leg_r2_joint": 0,
        "leg_r3_joint": 0,
        "leg_r4_joint": 0,
        "leg_r5_joint": 0, # ankle
        "leg_r6_joint": 0, # ankle
    }

    leg_dof_damping = {
        "leg_l1_joint": 0,
        "leg_l2_joint": 0,
        "leg_l3_joint": 0,
        "leg_l4_joint": 0,
        "leg_l5_joint": 0, # ankle
        "leg_l6_joint": 0, # ankle
        "leg_r1_joint": 0,
        "leg_r2_joint": 0,
        "leg_r3_joint": 0,
        "leg_r4_joint": 0,
        "leg_r5_joint": 0, # ankle
        "leg_r6_joint": 0, # ankle
    }
else: # 混合控制模式CSP
    leg_dof_stiffness = {
        "leg_l1_joint": 200,
        "leg_l2_joint": 200,
        "leg_l3_joint": 350,
        "leg_l4_joint": 350,
        "leg_l5_joint": 200, # ankle
        "leg_l6_joint": 200, # ankle
        "leg_r1_joint": 200,
        "leg_r2_joint": 200,
        "leg_r3_joint": 350,
        "leg_r4_joint": 350,
        "leg_r5_joint": 200, # ankle
        "leg_r6_joint": 200, # ankle
    }

    leg_dof_damping = {
        "leg_l1_joint": 30,
        "leg_l2_joint": 30,
        "leg_l3_joint": 30,
        "leg_l4_joint": 30,
        "leg_l5_joint": 10, # ankle
        "leg_l6_joint": 10, # ankle
        "leg_r1_joint": 30,
        "leg_r2_joint": 30,
        "leg_r3_joint": 30,
        "leg_r4_joint": 30,
        "leg_r5_joint": 10, # ankle
        "leg_r6_joint": 10, # ankle
    }

if TAU_ARM_FLAG: # 纯力矩控制模式
# 为手臂和腿部分别创建Kp和Kd字典
    arm_dof_stiffness = {
        "zarm_l1_joint": 0,
        "zarm_l2_joint": 0,
        "zarm_l3_joint": 0,
        "zarm_l4_joint": 0,
        "zarm_l5_joint": 0,
        "zarm_l6_joint": 0,
        "zarm_l7_joint": 0,
        "zarm_r1_joint": 0,
        "zarm_r2_joint": 0,
        "zarm_r3_joint": 0,
        "zarm_r4_joint": 0,
        "zarm_r5_joint": 0,
        "zarm_r6_joint": 0,
        "zarm_r7_joint": 0,
    }

    arm_dof_damping = {
        "zarm_l1_joint": 0,
        "zarm_l2_joint": 0,
        "zarm_l3_joint": 0,
        "zarm_l4_joint": 0,
        "zarm_l5_joint": 0,
        "zarm_l6_joint": 0,
        "zarm_l7_joint": 0,
        "zarm_r1_joint": 0,
        "zarm_r2_joint": 0,
        "zarm_r3_joint": 0,
        "zarm_r4_joint": 0,
        "zarm_r5_joint": 0,
        "zarm_r6_joint": 0,
        "zarm_r7_joint": 0,
    }
else: # 速度控制模式
    # 为手臂和腿部分别创建Kp和Kd字典
    arm_dof_stiffness = {
        "zarm_l1_joint": 0,
        "zarm_l2_joint": 0,
        "zarm_l3_joint": 0,
        "zarm_l4_joint": 0,
        "zarm_l5_joint": 0,
        "zarm_l6_joint": 0,
        "zarm_l7_joint": 0,
        "zarm_r1_joint": 0,
        "zarm_r2_joint": 0,
        "zarm_r3_joint": 0,
        "zarm_r4_joint": 0,
        "zarm_r5_joint": 0,
        "zarm_r6_joint": 0,
        "zarm_r7_joint": 0,
    }

    arm_dof_damping = {
        "zarm_l1_joint": 8,
        "zarm_l2_joint": 8,
        "zarm_l3_joint": 8,
        "zarm_l4_joint": 8,
        "zarm_l5_joint": 8,
        "zarm_l6_joint": 8,
        "zarm_l7_joint": 8,
        "zarm_r1_joint": 8,
        "zarm_r2_joint": 8,
        "zarm_r3_joint": 8,
        "zarm_r4_joint": 8,
        "zarm_r5_joint": 8,
        "zarm_r6_joint": 8,
        "zarm_r7_joint": 8,
    }   

@dataclass
class DefaultStateCfg:
    # -- limits
    effort_limit: float = 0
    """Maximum effort limit for the actuator. """
    # -- gains
    stiffness: float = 0
    damping: float = 0
    # -- state
    dof_pos: float = 0
    dof_vel: float = 0
    dof_effort: float = 0


KuavoCfg = {
    "arm_names": {
        "zarm_l1_joint",
        "zarm_l2_joint",
        "zarm_l3_joint",
        "zarm_l4_joint",
        "zarm_l5_joint",
        "zarm_l6_joint",
        "zarm_l7_joint",
        "zarm_r1_joint",
        "zarm_r2_joint",
        "zarm_r3_joint",
        "zarm_r4_joint",
        "zarm_r5_joint",
        "zarm_r6_joint",
        "zarm_r7_joint",
    },
    "leg_names": {
        "leg_l1_joint",
        "leg_l2_joint",
        "leg_l3_joint",
        "leg_l4_joint",
        "leg_l5_joint",
        "leg_l6_joint",
        "leg_r1_joint",
        "leg_r2_joint",
        "leg_r3_joint",
        "leg_r4_joint",
        "leg_r5_joint",
        "leg_r6_joint",
    },
    "default_state": {
        "zarm_l1_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_l1_joint"],
            damping=arm_dof_damping["zarm_l1_joint"],
            dof_pos=arm_initial_pos[0],
        ),
        "zarm_l2_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_l2_joint"],
            damping=arm_dof_damping["zarm_l2_joint"],
            dof_pos=arm_initial_pos[1],
        ),
        "zarm_l3_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_l3_joint"],
            damping=arm_dof_damping["zarm_l3_joint"],
            dof_pos=arm_initial_pos[2],
        ),
        "zarm_l4_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_l4_joint"],
            damping=arm_dof_damping["zarm_l4_joint"],
            dof_pos=arm_initial_pos[3],
        ),
        "zarm_l5_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_l5_joint"],
            damping=arm_dof_damping["zarm_l5_joint"],
            dof_pos=arm_initial_pos[4],
        ),
        "zarm_l6_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_l6_joint"],
            damping=arm_dof_damping["zarm_l6_joint"],
            dof_pos=arm_initial_pos[5],
        ),
        "zarm_l7_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_l7_joint"],
            damping=arm_dof_damping["zarm_l7_joint"],
            dof_pos=arm_initial_pos[6],
        ),
        "zarm_r1_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_r1_joint"],
            damping=arm_dof_damping["zarm_r1_joint"],
            dof_pos=arm_initial_pos[7],
        ),
        "zarm_r2_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_r2_joint"],
            damping=arm_dof_damping["zarm_r2_joint"],
            dof_pos=arm_initial_pos[8],
        ),
        "zarm_r3_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_r3_joint"],
            damping=arm_dof_damping["zarm_r3_joint"],
            dof_pos=arm_initial_pos[9],
        ),
        "zarm_r4_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_r4_joint"],
            damping=arm_dof_damping["zarm_r4_joint"],
            dof_pos=arm_initial_pos[10],
        ),
        "zarm_r5_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_r5_joint"],
            damping=arm_dof_damping["zarm_r5_joint"],
            dof_pos=arm_initial_pos[11],
        ),
        "zarm_r6_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_r6_joint"],
            damping=arm_dof_damping["zarm_r6_joint"],
            dof_pos=arm_initial_pos[12],
        ),
        "zarm_r7_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_r7_joint"],
            damping=arm_dof_damping["zarm_r7_joint"],
            dof_pos=arm_initial_pos[13],
        ),
        "leg_l1_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_l1_joint"],
            damping=leg_dof_damping["leg_l1_joint"],
            dof_pos=leg_initial_pos[0],
        ),
        "leg_l2_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_l2_joint"],
            damping=leg_dof_damping["leg_l2_joint"],
            dof_pos=leg_initial_pos[1],
        ),
        "leg_l3_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_l3_joint"],
            damping=leg_dof_damping["leg_l3_joint"],
            dof_pos=leg_initial_pos[2],
        ),
        "leg_l4_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_l4_joint"],
            damping=leg_dof_damping["leg_l4_joint"],
            dof_pos=leg_initial_pos[3],
        ),
        "leg_l5_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_l5_joint"],
            damping=leg_dof_damping["leg_l5_joint"],
            dof_pos=leg_initial_pos[4],
        ),
        "leg_l6_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_l6_joint"],
            damping=leg_dof_damping["leg_l6_joint"],
            dof_pos=leg_initial_pos[5],
        ),
        "leg_r1_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_r1_joint"],
            damping=leg_dof_damping["leg_r1_joint"],
            dof_pos=leg_initial_pos[6],
        ),
        "leg_r2_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_r2_joint"],
            damping=leg_dof_damping["leg_r2_joint"],
            dof_pos=leg_initial_pos[7],
        ),
        "leg_r3_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_r3_joint"],
            damping=leg_dof_damping["leg_r3_joint"],
            dof_pos=leg_initial_pos[8],
        ),
        "leg_r4_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_r4_joint"],
            damping=leg_dof_damping["leg_r4_joint"],
            dof_pos=leg_initial_pos[9],
        ),
        "leg_r5_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_r5_joint"],
            damping=leg_dof_damping["leg_r5_joint"],
            dof_pos=leg_initial_pos[10],
        ),
        "leg_r6_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_r6_joint"],
            damping=leg_dof_damping["leg_r6_joint"],
            dof_pos=leg_initial_pos[11],
        ),
    },
}
