import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data

try:
    import serial
except ImportError:
    serial = None


# ============================================================
# CONFIGURACIÓN GENERAL
# ============================================================

DT = 1.0 / 60.0

CART_SPEED = 0.55

MOTOR_FORCE = 80.0
MOTOR_MAX_VELOCITY = 2.0

SMOOTHING = 0.16


# Límites de seguridad del movimiento cartesiano
X_LIMITS = (-0.15, 0.95)
Y_LIMITS = (-0.75, 0.75)
Z_LIMITS = (0.05, 0.95)


# ============================================================
# FUNCIONES AUXILIARES
# ============================================================

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def deadzone(value, zone=0.08):

    if abs(value) < zone:
        return 0.0

    if value > 0:
        return (value - zone) / (1.0 - zone)

    return (value + zone) / (1.0 - zone)


# ============================================================
# BUSCAR URDF DE BAXTER
# ============================================================

def find_baxter_urdf():

    candidates = [

        Path(
            "vendor/pybullet_robots/"
            "data/baxter_common/"
            "baxter_description/"
            "urdf/toms_baxter.urdf"
        ),

        Path(
            "../vendor/pybullet_robots/"
            "data/baxter_common/"
            "baxter_description/"
            "urdf/toms_baxter.urdf"
        ),

        Path(
            "pybullet_robots/"
            "data/baxter_common/"
            "baxter_description/"
            "urdf/toms_baxter.urdf"
        ),

        Path(
            "data/baxter_common/"
            "baxter_description/"
            "urdf/toms_baxter.urdf"
        )
    ]

    # También permite utilizar:
    #
    # $env:BAXTER_URDF="C:\ruta\toms_baxter.urdf"

    env_urdf = os.getenv("BAXTER_URDF")

    if env_urdf:
        candidates.insert(0, Path(env_urdf))

    for path in candidates:

        if path.is_file():
            return path.resolve()

    return None


# ============================================================
# CONFIGURAR RUTAS DE PYBULLET
# ============================================================

def configure_search_path(urdf_path):

    """
    Configura las rutas donde PyBullet buscará:

    - plane.urdf
    - meshes
    - Baxter
    - baxter_description
    """

    data_root = urdf_path.parents[3]

    baxter_packages = urdf_path.parents[2]

    p.setAdditionalSearchPath(
        str(data_root)
    )

    p.setAdditionalSearchPath(
        str(baxter_packages)
    )

    p.setAdditionalSearchPath(
        pybullet_data.getDataPath()
    )


# ============================================================
# CLASE BAXTER
# ============================================================

class Baxter:

    def __init__(self, urdf_path):

        self.urdf_path = urdf_path

        configure_search_path(
            urdf_path
        )

        # ----------------------------------------------------
        # CARGAR BAXTER
        # ----------------------------------------------------

        self.robot = p.loadURDF(

            str(urdf_path),

            [0.5, -0.8, 0.0],

            [0.0, 0.0, -1.0, -1.0],

            useFixedBase=True
        )

        # ----------------------------------------------------
        # INFORMACIÓN DE JOINTS
        # ----------------------------------------------------

        self.joints = []

        self.joint_by_name = {}

        self.links = {}

        number_joints = p.getNumJoints(
            self.robot
        )

        print()
        print("====================================")
        print("JOINTS DE BAXTER")
        print("====================================")

        for i in range(number_joints):

            info = p.getJointInfo(
                self.robot,
                i
            )

            joint_name = info[1].decode(
                "utf-8"
            )

            joint_type = info[2]

            q_index = info[3]

            link_name = info[12].decode(
                "utf-8"
            )

            self.joints.append({

                "index": i,

                "name": joint_name,

                "link_name": link_name,

                "type": joint_type,

                "q_index": q_index,

                "lower": info[8],

                "upper": info[9],

                "max_force": info[10],

                "max_velocity": info[11]

            })

            self.joint_by_name[
                joint_name
            ] = i

            self.links[
                link_name
            ] = i

            print(
                i,
                joint_name,
                "qIndex:",
                q_index,
                "link:",
                link_name
            )

        print("====================================")
        print()

        # ----------------------------------------------------
        # ARTICULACIONES BRAZO IZQUIERDO
        # ----------------------------------------------------

        self.arm_joints = {

            "left": self.get_named_joints([

                "left_s0",
                "left_s1",
                "left_e0",
                "left_e1",
                "left_w0",
                "left_w1",
                "left_w2"

            ]),

            "right": self.get_named_joints([

                "right_s0",
                "right_s1",
                "right_e0",
                "right_e1",
                "right_w0",
                "right_w1",
                "right_w2"

            ])
        }

        # ----------------------------------------------------
        # EFECTORES FINALES
        # ----------------------------------------------------

        self.end_effectors = {

            "left":
                self.links.get(
                    "left_gripper",
                    48
                ),

            "right":
                self.links.get(
                    "right_gripper",
                    19
                )
        }

        # ----------------------------------------------------
        # PINZAS
        # ----------------------------------------------------

        self.gripper_joints = {

            "left": self.get_named_joints([

                "l_gripper_l_finger_joint",
                "l_gripper_r_finger_joint"

            ]),

            "right": self.get_named_joints([

                "r_gripper_l_finger_joint",
                "r_gripper_r_finger_joint"

            ])
        }

        # ----------------------------------------------------
        # BRAZO ACTIVO
        # ----------------------------------------------------

        self.active_arm = "left"

        self.grip_closed = False

        # ----------------------------------------------------
        # POSICIÓN OBJETIVO
        # ----------------------------------------------------

        self.targets = {}

        for arm in ["left", "right"]:

            position = p.getLinkState(

                self.robot,

                self.end_effectors[arm],

                computeForwardKinematics=True

            )[4]

            self.targets[arm] = np.array(

                position,

                dtype=float

            )

        # ----------------------------------------------------
        # OBJETO
        # ----------------------------------------------------

        self.object_id = None

        self.target_id = None

        self.grasp_constraint = None

        # ----------------------------------------------------
        # ESCENA
        # ----------------------------------------------------

        self.create_scene()

        self.set_gripper(
            False
        )

    # ========================================================
    # OBTENER JOINTS POR NOMBRE
    # ========================================================

    def get_named_joints(self, names):

        result = []

        for name in names:

            if name in self.joint_by_name:

                result.append(
                    self.joint_by_name[name]
                )

        return result

    # ========================================================
    # CREAR ESCENA
    # ========================================================

    def create_scene(self):

        p.setGravity(
            0,
            0,
            -9.81
        )

        p.setTimeStep(
            DT
        )

        p.setPhysicsEngineParameter(
            numSolverIterations=100
        )

        # ----------------------------------------------------
        # PLANO
        # ----------------------------------------------------

        p.loadURDF(
            "plane.urdf",
            [0, 0, -1],
            useFixedBase=True
        )

        # ----------------------------------------------------
        # POSICIÓN ACTUAL DE LA MANO
        # ----------------------------------------------------

        hand_position = np.array(

            p.getLinkState(

                self.robot,

                self.end_effectors[
                    "left"
                ],

                computeForwardKinematics=True

            )[4]

        )

        # ----------------------------------------------------
        # CREAR CUBO
        # ----------------------------------------------------

        self.object_start = np.array([

            hand_position[0],

            hand_position[1],

            max(
                0.08,
                hand_position[2] - 0.22
            )

        ])

        self.object_id = p.loadURDF(

            "cube_small.urdf",

            self.object_start.tolist(),

            useFixedBase=False
        )

        # ----------------------------------------------------
        # DESTINO
        # ----------------------------------------------------

        self.target_pos = (

            self.object_start +

            np.array([
                0.30,
                -0.10,
                0.0
            ])

        )

        # ----------------------------------------------------
        # VISUAL DESTINO
        # ----------------------------------------------------

        visual = p.createVisualShape(

            p.GEOM_BOX,

            halfExtents=[
                0.035,
                0.035,
                0.035
            ],

            rgbaColor=[
                0.2,
                0.9,
                0.2,
                0.7
            ]
        )

        collision = p.createCollisionShape(

            p.GEOM_BOX,

            halfExtents=[
                0.035,
                0.035,
                0.035
            ]
        )

        self.target_id = p.createMultiBody(

            baseMass=0,

            baseCollisionShapeIndex=
                collision,

            baseVisualShapeIndex=
                visual,

            basePosition=
                self.target_pos.tolist()
        )

        # ----------------------------------------------------
        # TEXTO
        # ----------------------------------------------------

        p.addUserDebugText(

            "OBJETO",

            self.object_start.tolist(),

            textColorRGB=[
                1,
                0.2,
                0.2
            ],

            textSize=1.2
        )

        p.addUserDebugText(

            "DESTINO",

            self.target_pos.tolist(),

            textColorRGB=[
                0.2,
                1,
                0.2
            ],

            textSize=1.2
        )

    # ========================================================
    # CONTROLAR PINZA
    # ========================================================

    def _set_gripper(
        self,
        arm,
        closed
    ):

        joints = self.gripper_joints[
            arm
        ]

        if not joints:

            return

        # ----------------------------------------------------
        # PINZA CERRADA
        # ----------------------------------------------------

        if closed:

            values = [
                0.0,
                0.0
            ]

        # ----------------------------------------------------
        # PINZA ABIERTA
        # ----------------------------------------------------

        else:

            values = [
                0.020,
                -0.020
            ]

        # ----------------------------------------------------
        # APLICAR
        # ----------------------------------------------------

        for joint, value in zip(
            joints,
            values
        ):

            info = p.getJointInfo(

                self.robot,

                joint
            )

            lower = info[8]

            upper = info[9]

            value = clamp(
                value,
                lower,
                upper
            )

            p.setJointMotorControl2(

                self.robot,

                joint,

                p.POSITION_CONTROL,

                targetPosition=value,

                force=20,

                maxVelocity=2.0
            )

    # ========================================================
    # ABRIR/CERRAR
    # ========================================================

    def set_gripper(
        self,
        closed
    ):

        self.grip_closed = bool(
            closed
        )

        self._set_gripper(

            self.active_arm,

            self.grip_closed
        )

        if self.grip_closed:

            self.try_grasp()

        else:

            self.release()

    # ========================================================
    # TOGGLE
    # ========================================================

    def toggle_gripper(self):

        self.set_gripper(
            not self.grip_closed
        )

    # ========================================================
    # INTENTAR AGARRAR
    # ========================================================

    def try_grasp(self):

        if self.grasp_constraint is not None:

            return

        ee = self.end_effectors[
            self.active_arm
        ]

        hand = np.array(

            p.getLinkState(

                self.robot,

                ee,

                computeForwardKinematics=True

            )[4]

        )

        object_position = np.array(

            p.getBasePositionAndOrientation(

                self.object_id

            )[0]

        )

        distance = np.linalg.norm(

            hand -
            object_position

        )

        if distance < 0.13:

            self.grasp_constraint = (

                p.createConstraint(

                    self.robot,

                    ee,

                    self.object_id,

                    -1,

                    p.JOINT_FIXED,

                    [0, 0, 0],

                    [0, 0, 0],

                    [0, 0, 0]
                )
            )

            print()
            print(
                ">>> OBJETO AGARRADO <<<"
            )
            print()

    # ========================================================
    # SOLTAR
    # ========================================================

    def release(self):

        if self.grasp_constraint is not None:

            p.removeConstraint(

                self.grasp_constraint
            )

            self.grasp_constraint = None

            print()
            print(
                ">>> OBJETO LIBERADO <<<"
            )
            print()

    # ========================================================
    # HOME
    # ========================================================

    def home(self):

        position = p.getLinkState(

            self.robot,

            self.end_effectors[
                self.active_arm
            ],

            computeForwardKinematics=True

        )[4]

        self.targets[
            self.active_arm
        ] = np.array(

            position,

            dtype=float
        )

    # ========================================================
    # CINEMÁTICA INVERSA
    # ========================================================

    def calculate_ik(
        self,
        arm,
        target
    ):

        end_effector = (

            self.end_effectors[
                arm
            ]
        )

        # ----------------------------------------------------
        # OBTENER RANGOS DE LOS DOF
        # ----------------------------------------------------

        lower_limits = []

        upper_limits = []

        joint_ranges = []

        rest_poses = []

        movable_joints = []

        for info in self.joints:

            if (

                info["q_index"] >= 0

                and

                info["type"] in (

                    p.JOINT_REVOLUTE,

                    p.JOINT_PRISMATIC
                )

            ):

                movable_joints.append(
                    info
                )

                lower = info["lower"]

                upper = info["upper"]

                # Algunos joints pueden no tener
                # límites útiles.

                if upper <= lower:

                    lower = -2.0

                    upper = 2.0

                lower_limits.append(
                    -2.0
                )

                upper_limits.append(
                    2.0
                )

                joint_ranges.append(
                    4.0
                )

                rest_poses.append(

                    p.getJointState(

                        self.robot,

                        info["index"]

                    )[0]
                )

        # ----------------------------------------------------
        # CALCULAR IK
        # ----------------------------------------------------

        try:

            joint_poses = (

                p.calculateInverseKinematics(

                    self.robot,

                    end_effector,

                    target.tolist(),

                    lowerLimits=
                        lower_limits,

                    upperLimits=
                        upper_limits,

                    jointRanges=
                        joint_ranges,

                    restPoses=
                        rest_poses,

                    maxNumIterations=50,

                    residualThreshold=1e-4

                )
            )

        except Exception:

            joint_poses = (

                p.calculateInverseKinematics(

                    self.robot,

                    end_effector,

                    target.tolist(),

                    maxNumIterations=50

                )
            )

        return joint_poses

    # ========================================================
    # APLICAR IK
    # ========================================================

    def apply_target(
        self,
        arm
    ):

        target = self.targets[
            arm
        ]

        joint_poses = self.calculate_ik(

            arm,

            target
        )

        # ----------------------------------------------------
        # IMPORTANTE:
        #
        # Baxter tiene muchos joints.
        #
        # calculateInverseKinematics()
        # devuelve solamente los DOF.
        #
        # El repositorio original usa:
        #
        # jointPoses[qIndex - 7]
        #
        # ----------------------------------------------------

        for info in self.joints:

            q_index = info[
                "q_index"
            ]

            if q_index < 0:

                continue

            index = q_index - 7

            if index < 0:

                continue

            if index >= len(
                joint_poses
            ):

                continue

            # ------------------------------------------------
            # SOLO CONTROLAR EL BRAZO ACTIVO
            # ------------------------------------------------

            if (

                info["index"]

                in

                self.arm_joints[arm]

            ):

                target_position = (

                    joint_poses[index]
                )

                p.setJointMotorControl2(

                    bodyIndex=
                        self.robot,

                    jointIndex=
                        info["index"],

                    controlMode=
                        p.POSITION_CONTROL,

                    targetPosition=
                        target_position,

                    force=
                        MOTOR_FORCE,

                    maxVelocity=
                        MOTOR_MAX_VELOCITY
                )

        # ----------------------------------------------------
        # CONTROLAR PINZA
        # ----------------------------------------------------

        self._set_gripper(

            arm,

            self.grip_closed
        )

    # ========================================================
    # MOVIMIENTO CARTESIANO
    # ========================================================

    def move_cartesian(

        self,
        vx,
        vy,
        vz

    ):

        arm = self.active_arm

        target = self.targets[
            arm
        ]

        # ----------------------------------------------------
        # X
        # ----------------------------------------------------

        target[0] += (
            vx * DT
        )

        # ----------------------------------------------------
        # Y
        # ----------------------------------------------------

        target[1] += (
            vy * DT
        )

        # ----------------------------------------------------
        # Z
        # ----------------------------------------------------

        target[2] += (
            vz * DT
        )

        # ----------------------------------------------------
        # LIMITES
        # ----------------------------------------------------

        target[0] = clamp(

            target[0],

            X_LIMITS[0],

            X_LIMITS[1]
        )

        target[1] = clamp(

            target[1],

            Y_LIMITS[0],

            Y_LIMITS[1]
        )

        target[2] = clamp(

            target[2],

            Z_LIMITS[0],

            Z_LIMITS[1]
        )

    # ========================================================
    # CAMBIAR BRAZO
    # ========================================================

    def switch_arm(self):

        if self.active_arm == "left":

            self.active_arm = "right"

        else:

            self.active_arm = "left"

        self.grip_closed = False

        self.home()

        print()
        print(
            "BRAZO ACTIVO:",
            self.active_arm.upper()
        )
        print()

    # ========================================================
    # MOVIMIENTO SUAVE
    # ========================================================

    def move_to(

        self,
        waypoint,
        duration=1.0

    ):

        arm = self.active_arm

        start = np.array(

            self.targets[arm],

            dtype=float
        )

        steps = max(

            1,

            int(
                duration / DT
            )
        )

        for i in range(
            steps
        ):

            progress = (

                (i + 1)
                /
                steps
            )

            # Smoothstep
            smooth = (

                progress
                *
                progress
                *
                (
                    3
                    -
                    2 *
                    progress
                )
            )

            self.targets[arm] = (

                start *
                (1 - smooth)

                +

                waypoint *
                smooth
            )

            self.apply_target(
                arm
            )

            p.stepSimulation()

            time.sleep(
                DT
            )

    # ========================================================
    # PICK & PLACE
    # ========================================================

    def pick_and_place(self):

        arm = self.active_arm

        object_position = np.array(

            p.getBasePositionAndOrientation(

                self.object_id

            )[0]

        )

        target_position = np.array(

            self.target_pos
        )

        print()
        print(
            "================================"
        )
        print(
            "INICIANDO PICK & PLACE"
        )
        print(
            "================================"
        )

        # ----------------------------------------------------
        # 1. ACERCARSE
        # ----------------------------------------------------

        waypoint_1 = np.array([

            object_position[0],

            object_position[1],

            object_position[2]
            + 0.18
        ])

        self.grip_closed = False

        self._set_gripper(
            arm,
            False
        )

        self.move_to(

            waypoint_1,

            duration=1.0
        )

        # ----------------------------------------------------
        # 2. BAJAR
        # ----------------------------------------------------

        waypoint_2 = np.array([

            object_position[0],

            object_position[1],

            object_position[2]
            + 0.07
        ])

        self.move_to(

            waypoint_2,

            duration=1.0
        )

        # ----------------------------------------------------
        # 3. CERRAR
        # ----------------------------------------------------

        self.grip_closed = True

        self._set_gripper(

            arm,

            True
        )

        time.sleep(
            0.5
        )

        self.try_grasp()

        # ----------------------------------------------------
        # 4. LEVANTAR
        # ----------------------------------------------------

        waypoint_3 = np.array([

            object_position[0],

            object_position[1],

            object_position[2]
            + 0.20
        ])

        self.move_to(

            waypoint_3,

            duration=1.0
        )

        # ----------------------------------------------------
        # 5. MOVER AL DESTINO
        # ----------------------------------------------------

        waypoint_4 = np.array([

            target_position[0],

            target_position[1],

            target_position[2]
            + 0.20
        ])

        self.move_to(

            waypoint_4,

            duration=1.2
        )

        # ----------------------------------------------------
        # 6. BAJAR
        # ----------------------------------------------------

        waypoint_5 = np.array([

            target_position[0],

            target_position[1],

            target_position[2]
            + 0.08
        ])

        self.move_to(

            waypoint_5,

            duration=0.8
        )

        # ----------------------------------------------------
        # 7. SOLTAR
        # ----------------------------------------------------

        self.grip_closed = False

        self._set_gripper(

            arm,

            False
        )

        self.release()

        # ----------------------------------------------------
        # 8. SUBIR
        # ----------------------------------------------------

        waypoint_6 = np.array([

            target_position[0],

            target_position[1],

            target_position[2]
            + 0.20
        ])

        self.move_to(

            waypoint_6,

            duration=0.8
        )

        print()
        print(
            "================================"
        )
        print(
            "PICK & PLACE TERMINADO"
        )
        print(
            "================================"
        )
        print()

    # ========================================================
    # CONTROL DE TECLADO
    # ========================================================

    def keyboard_control(self):

        keys = p.getKeyboardEvents()

        vx = 0.0

        vy = 0.0

        vz = 0.0

        # ----------------------------------------------------
        # X
        # ----------------------------------------------------

        if (

            p.B3G_LEFT_ARROW in keys

            and

            keys[
                p.B3G_LEFT_ARROW
            ]

            &

            p.KEY_IS_DOWN

        ):

            vx -= CART_SPEED

        if (

            p.B3G_RIGHT_ARROW in keys

            and

            keys[
                p.B3G_RIGHT_ARROW
            ]

            &

            p.KEY_IS_DOWN

        ):

            vx += CART_SPEED

        # ----------------------------------------------------
        # Y
        # ----------------------------------------------------

        if (

            p.B3G_UP_ARROW in keys

            and

            keys[
                p.B3G_UP_ARROW
            ]

            &

            p.KEY_IS_DOWN

        ):

            vy += CART_SPEED

        if (

            p.B3G_DOWN_ARROW in keys

            and

            keys[
                p.B3G_DOWN_ARROW
            ]

            &

            p.KEY_IS_DOWN

        ):

            vy -= CART_SPEED

        # ----------------------------------------------------
        # Z
        # ----------------------------------------------------

        if (

            ord("w") in keys

            and

            keys[
                ord("w")
            ]

            &

            p.KEY_IS_DOWN

        ):

            vz += CART_SPEED

        if (

            ord("s") in keys

            and

            keys[
                ord("s")
            ]

            &

            p.KEY_IS_DOWN

        ):

            vz -= CART_SPEED

        # ----------------------------------------------------
        # MOVIMIENTO
        # ----------------------------------------------------

        if (

            vx != 0
            or
            vy != 0
            or
            vz != 0

        ):

            self.move_cartesian(

                vx,
                vy,
                vz
            )

        # ----------------------------------------------------
        # G = TOGGLE PINZA
        # ----------------------------------------------------

        if (

            ord("g") in keys

            and

            keys[
                ord("g")
            ]

            &

            p.KEY_WAS_TRIGGERED

        ):

            self.toggle_gripper()

        # ----------------------------------------------------
        # O = ABRIR
        # ----------------------------------------------------

        if (

            ord("o") in keys

            and

            keys[
                ord("o")
            ]

            &

            p.KEY_WAS_TRIGGERED

        ):

            self.set_gripper(
                False
            )

        # ----------------------------------------------------
        # C = CERRAR
        # ----------------------------------------------------

        if (

            ord("c") in keys

            and

            keys[
                ord("c")
            ]

            &

            p.KEY_WAS_TRIGGERED

        ):

            self.set_gripper(
                True
            )

        # ----------------------------------------------------
        # TAB
        #
        # IMPORTANTE:
        #
        # PyBullet NO tiene p.B3G_TAB.
        #
        # TAB = ASCII 9
        # ----------------------------------------------------

        if (

            9 in keys

            and

            keys[9]

            &

            p.KEY_WAS_TRIGGERED

        ):

            self.switch_arm()

        # ----------------------------------------------------
        # H = HOME
        # ----------------------------------------------------

        if (

            ord("h") in keys

            and

            keys[
                ord("h")
            ]

            &

            p.KEY_WAS_TRIGGERED

        ):

            self.home()

            print(
                "HOME"
            )

        # ----------------------------------------------------
        # P = PICK & PLACE
        # ----------------------------------------------------

        if (

            ord("p") in keys

            and

            keys[
                ord("p")
            ]

            &

            p.KEY_WAS_TRIGGERED

        ):

            self.pick_and_place()

        # ----------------------------------------------------
        # ESC
        #
        # ESC = ASCII 27
        # ----------------------------------------------------

        if (

            27 in keys

            and

            keys[27]

            &

            p.KEY_WAS_TRIGGERED

        ):

            p.disconnect()


# ============================================================
# CONTROL SERIAL ESP32
# ============================================================

class SerialConsole:

    def __init__(
        self,
        port=None,
        baud=115200
    ):

        self.ser = None

        self.grip = False

        self.arm = 0

        if port is None:

            return

        if serial is None:

            print()
            print(
                "ERROR: pyserial no está instalado."
            )

            print(
                "Instala con:"
            )

            print(
                "pip install pyserial"
            )

            print()

            return

        try:

            self.ser = serial.Serial(

                port,

                baud,

                timeout=0
            )

            time.sleep(
                2
            )

            print()
            print(
                "================================"
            )
            print(
                "ESP32 CONECTADO"
            )
            print(
                "Puerto:",
                port
            )
            print(
                "Baudrate:",
                baud
            )
            print(
                "================================"
            )
            print()

        except Exception as error:

            print()
            print(
                "No se pudo abrir:",
                port
            )

            print(
                "Error:",
                error
            )

            print(
                "Se utilizará teclado."
            )

            print()

    # ========================================================
    # LEER ESP32
    # ========================================================

    def read(self):

        if self.ser is None:

            return None

        latest = None

        while self.ser.in_waiting:

            line = (

                self.ser
                .readline()
                .decode(
                    errors="ignore"
                )
                .strip()
            )

            if line:

                latest = line

        if latest is None:

            return None

        # ----------------------------------------------------
        # FORMATO:
        #
        # V,x,y,z,grip,arm
        # ----------------------------------------------------

        parts = latest.split(",")

        if len(parts) != 6:

            return None

        if parts[0] != "V":

            return None

        try:

            x = float(
                parts[1]
            )

            y = float(
                parts[2]
            )

            z = float(
                parts[3]
            )

            grip = int(
                parts[4]
            )

            arm = int(
                parts[5]
            )

            return (

                x,
                y,
                z,
                bool(grip),
                bool(arm)

            )

        except ValueError:

            return None


# ============================================================
# MOSTRAR CONTROLES
# ============================================================

def print_controls():

    print()
    print(
        "============================================"
    )
    print(
        "       CONTROL BAXTER - PYBULLET"
    )
    print(
        "============================================"
    )

    print()

    print(
        "TECLADO:"
    )

    print(
        "  Flecha IZQ / DER -> X"
    )

    print(
        "  Flecha ARRIBA / ABAJO -> Y"
    )

    print(
        "  W / S -> Z"
    )

    print(
        "  G -> Abrir/Cerrar pinza"
    )

    print(
        "  O -> Abrir pinza"
    )

    print(
        "  C -> Cerrar pinza"
    )

    print(
        "  TAB -> Cambiar brazo"
    )

    print(
        "  H -> Home"
    )

    print(
        "  P -> Pick & Place"
    )

    print(
        "  ESC -> Salir"
    )

    print()

    print(
        "ESP32:"
    )

    print(
        "  Joystick X -> movimiento X"
    )

    print(
        "  Joystick Y -> movimiento Y"
    )

    print(
        "  Potenciómetro -> altura Z"
    )

    print(
        "  Botón GRIP -> pinza"
    )

    print(
        "  Botón ARM -> brazo"
    )

    print()

    print(
        "============================================"
    )

    print()


# ============================================================
# MAIN
# ============================================================

def main():

    # --------------------------------------------------------
    # ARGUMENTOS
    # --------------------------------------------------------

    parser = argparse.ArgumentParser()

    parser.add_argument(

        "--port",

        default=None,

        help="Puerto COM del ESP32, ejemplo COM5"

    )

    parser.add_argument(

        "--baud",

        type=int,

        default=115200

    )

    args = parser.parse_args()

    # --------------------------------------------------------
    # BUSCAR BAXTER
    # --------------------------------------------------------

    urdf = find_baxter_urdf()

    if urdf is None:

        print()

        print(
            "ERROR:"
        )

        print(
            "No se encontró:"
        )

        print(
            "toms_baxter.urdf"
        )

        print()

        print(
            "Debes tener:"
        )

        print(
            "vendor/pybullet_robots/"
            "data/baxter_common/"
            "baxter_description/"
            "urdf/toms_baxter.urdf"
        )

        print()

        sys.exit(1)

    # --------------------------------------------------------
    # CONECTAR PYBULLET
    # --------------------------------------------------------

    client = p.connect(
        p.GUI
    )

    if client < 0:

        raise RuntimeError(
            "No se pudo abrir PyBullet."
        )

    # --------------------------------------------------------
    # CONFIGURAR VENTANA
    # --------------------------------------------------------

    p.configureDebugVisualizer(

        p.COV_ENABLE_GUI,

        0
    )

    p.resetDebugVisualizerCamera(

        cameraDistance=2.6,

        cameraYaw=180,

        cameraPitch=-15,

        cameraTargetPosition=[
            0.5,
            -0.25,
            0.55
        ]
    )

    # --------------------------------------------------------
    # CREAR BAXTER
    # --------------------------------------------------------

    robot = Baxter(
        urdf
    )

    # --------------------------------------------------------
    # SERIAL
    # --------------------------------------------------------

    console = SerialConsole(

        args.port,

        args.baud
    )

    # --------------------------------------------------------
    # MOSTRAR CONTROLES
    # --------------------------------------------------------

    print_controls()

    print(
        "URDF:"
    )

    print(
        urdf
    )

    print()

    print(
        "Brazo inicial:"
    )

    print(
        robot.active_arm.upper()
    )

    print()

    # --------------------------------------------------------
    # VARIABLES
    # --------------------------------------------------------

    last_arm = robot.active_arm

    last_grip = robot.grip_closed

    # --------------------------------------------------------
    # BUCLE PRINCIPAL
    # --------------------------------------------------------

    try:

        while p.isConnected():

            # ------------------------------------------------
            # LEER ESP32
            # ------------------------------------------------

            data = console.read()

            if data is not None:

                x, y, z, grip, right_arm = data

                # --------------------------------------------
                # BRAZO
                # --------------------------------------------

                selected_arm = (

                    "right"

                    if right_arm

                    else

                    "left"

                )

                if (

                    selected_arm

                    !=

                    robot.active_arm

                ):

                    robot.active_arm = (
                        selected_arm
                    )

                    robot.home()

                    robot.grip_closed = False

                # --------------------------------------------
                # PINZA
                # --------------------------------------------

                if (

                    grip

                    !=

                    robot.grip_closed

                ):

                    robot.set_gripper(
                        grip
                    )

                # --------------------------------------------
                # JOYSTICK
                # --------------------------------------------

                x = deadzone(
                    x
                )

                y = deadzone(
                    y
                )

                z = deadzone(
                    z
                )

                # --------------------------------------------
                # CONTROL CARTESIANO
                # --------------------------------------------

                robot.move_cartesian(

                    x * CART_SPEED,

                    -y * CART_SPEED,

                    z * CART_SPEED

                )

            # ------------------------------------------------
            # SI NO HAY ESP32
            # ------------------------------------------------

            else:

                robot.keyboard_control()

            # ------------------------------------------------
            # APLICAR TARGET
            # ------------------------------------------------

            robot.apply_target(

                robot.active_arm
            )

            # ------------------------------------------------
            # MOSTRAR CAMBIO DE BRAZO
            # ------------------------------------------------

            if (

                robot.active_arm

                !=

                last_arm

            ):

                print(

                    "Brazo:",
                    robot.active_arm.upper()

                )

                last_arm = (
                    robot.active_arm
                )

            # ------------------------------------------------
            # MOSTRAR CAMBIO DE PINZA
            # ------------------------------------------------

            if (

                robot.grip_closed

                !=

                last_grip

            ):

                state = (

                    "CERRADA"

                    if robot.grip_closed

                    else

                    "ABIERTA"

                )

                print(
                    "Pinza:",
                    state
                )

                last_grip = (
                    robot.grip_closed
                )

            # ------------------------------------------------
            # SIMULACIÓN
            # ------------------------------------------------

            p.stepSimulation()

            time.sleep(
                DT
            )

    except KeyboardInterrupt:

        print()

        print(
            "Programa detenido."
        )

    finally:

        if p.isConnected():

            p.disconnect()


# ============================================================
# EJECUTAR
# ============================================================

if __name__ == "__main__":

    main()