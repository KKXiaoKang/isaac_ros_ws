import math
from copy import deepcopy
import time
import os 

from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core.utils.stage import add_reference_to_stage
from scipy.spatial.transform import Rotation

from nio import ROOT_PATH
from nio.agent import Kuavo
from nio.camera import Sensor

ASSET = ROOT_PATH / "assets"

# SIM_CFG = {"decimation": 4, "physics_dt": 1 / 400}
SIM_CFG = {"decimation": 10, "physics_dt": 1 / 1000}

SENSOR_CFG = {"prim_path": "/World/Kuavo"}

"""
    40 - 4代短手短脚
    45 - 4pro长手 
"""
ROBOT_VERSION = int(os.getenv('ROBOT_VERSION', '45'))  # 45 作为默认值
print(f"Initializing robot with version: {ROBOT_VERSION}")

if ROBOT_VERSION == 40:
    AGENT_CFG = {
        "init_pos": [-0.00593, 0, 0.78],
        "init_roll": 0,
        "init_pitch": 0.0,
        "init_yaw": 0,
    }
elif ROBOT_VERSION == 45:
    AGENT_CFG = {
        "init_pos": [-0.00593, 0, 0.815],
        "init_roll": 0,
        "init_pitch": 0.0,
        "init_yaw": 0,
    }

"""
    0 - 车企检测
    1 - 比赛场景
"""
SCENE_FLAG = 1

if SCENE_FLAG == 0:
    SCENE_CFG = {
        "warehouse": {"position": [0, 0, 0], "orientation": [1, 0, 0, 0]},
        "car": {"position": [0.8283, 2.54758, 0.0], "orientation": [0, 0, -90]},
    }
elif SCENE_FLAG == 1:
    ROBOT_SCENE_POINT_INDEX = int(os.getenv('ROBOT_SCENE_POINT_INDEX', '3'))
    SCENE_CFG = {
        "point_1": {"position": [11.8283, -8.7758, 0.0], "orientation": [0, 0, -180]}, # 沙地前
        "point_2": {"position": [2.3283, 9.458, -0.051], "orientation": [0, 0, -90]},  # 导航场景
        "point_3": {"position": [9.5283, 8.4758, -0.052], "orientation": [0, 0, 0]},   # 搬运场景
    }

"""
    基础环境
"""
class Env:
    def __init__(self):
        self.warehouse = None
        self.car = None
        self.agent = None
        self.rsd455 = None
        self._load_scene()
        self._load_agent()
        self._load_sensor()
        self.sim_cfg = deepcopy(SIM_CFG)
        self.decimation = self.sim_cfg["decimation"]
        self.sim = SimulationContext(
            backend="numpy",
            physics_dt=self.sim_cfg["physics_dt"],
            rendering_dt=self.sim_cfg["physics_dt"] * self.sim_cfg["decimation"],
        )

    def reset(self):
        self.sim.reset()
        self.agent.initialize()
        self.rsd455.initialize()

    def step(self, action=None):
        for _ in range(self.decimation):
            if action is not None:
                self.agent.apply_action(action)
            self.sim.step(render=False)
        self.sim.render()

    def get_sensor(self):
        return self.rsd455

    def get_agent(self):
        return self.agent

    def _load_scene(self):
        cfg = deepcopy(SCENE_CFG)
        if SCENE_FLAG == 0:
            add_reference_to_stage(
                usd_path=str(ASSET / "car/car.usd"), prim_path="/World/Scene/car"
            )
            self.car = XFormPrim("/World/Scene/car")
            initial_quat = Rotation.from_euler(
                "xyz", cfg["car"]["orientation"], degrees=True
            ).as_quat()[[3, 0, 1, 2]]
            self.car.set_world_pose(cfg["car"]["position"], initial_quat)
            self.sphere1 = XFormPrim("/World/Scene/car/Sphere1")
        elif SCENE_FLAG == 1:
            add_reference_to_stage(
                usd_path=str(ASSET / "scene/scene.usd"), prim_path="/World"
            )
            self.car = XFormPrim("/World/Scene")
            
            # 根据ROBOT_SCENE_POINT_INDEX选择对应的点位
            point_key = f"point_{ROBOT_SCENE_POINT_INDEX}"
            if point_key not in cfg:
                print(f"Warning: {point_key} not found in SCENE_CFG, using point_1 as default")
                point_key = "point_1"
            
            selected_point = cfg[point_key]
            initial_quat = Rotation.from_euler(
                "xyz", selected_point["orientation"], degrees=True
            ).as_quat()[[3, 0, 1, 2]]
            self.car.set_world_pose(selected_point["position"], initial_quat)

    def _load_agent(self):
        agent_cfg = deepcopy(AGENT_CFG)
        roll = math.radians(agent_cfg["init_roll"])
        pitch = math.radians(agent_cfg["init_pitch"])
        yaw = math.radians(agent_cfg["init_yaw"])
        initial_quat = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_quat()[
            [3, 0, 1, 2]
        ]
        if ROBOT_VERSION == 40:
            usd_path = str(ASSET / "kuavo.usd")
        elif ROBOT_VERSION == 45:
            usd_path = str(ASSET / "biped_s45_my.usd")
        else:
            print(f"ROBOT_VERSION: {ROBOT_VERSION} not supported")
            exit()
        self.agent = Kuavo(usd_path)
        self.agent.set_world_pose(agent_cfg["init_pos"], initial_quat)

    def _load_sensor(self):
        snsor_cfg = deepcopy(SENSOR_CFG)
        self.rsd455 = Sensor(
            usd_path=str(ASSET / "rsd455.usd"), prim_path=snsor_cfg["prim_path"]
        )

    def shutdown(self) -> None:
        self.sim.stop()
