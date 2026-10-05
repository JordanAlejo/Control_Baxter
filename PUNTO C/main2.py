import pybullet as p
import pybullet_data
import time
import math


# ============================================================
# CONFIGURACIÓN
# ============================================================

URDF = "atlas/atlas_v4_with_multisense.urdf"

DT = 1.0 / 240.0

p.connect(p.GUI)

p.setAdditionalSearchPath(
    pybullet_data.getDataPath()
)

p.setGravity(
    0,
    0,
    -9.81
)

p.setTimeStep(DT)

p.setPhysicsEngineParameter(
    fixedTimeStep=DT,
    numSolverIterations=80,
    numSubSteps=1
)


# ============================================================
# PISO
# ============================================================

plane = p.loadURDF(
    "plane.urdf",
    [0, 0, 0]
)


# ============================================================
# ATLAS
# ============================================================

atlas = p.loadURDF(
    URDF,
    [0, 0, 1.15],
    useFixedBase=False
)


# ============================================================
# CONFIGURACIÓN DE CONTROL
# ============================================================

FUERZA = 250

KP = 0.25
KD = 0.08

BALANCE_KP = 0.7
BALANCE_KD = 0.12


# ============================================================
# JOINTS
# ============================================================

joints = {}

for i in range(
    p.getNumJoints(atlas)
):

    info = p.getJointInfo(
        atlas,
        i
    )

    nombre = info[1].decode(
        "utf-8"
    )

    joints[nombre] = i


# ============================================================
# FUNCIÓN JOINT
# ============================================================

def joint(nombre):

    return joints.get(
        nombre,
        None
    )


# ============================================================
# PIERNAS
# ============================================================

L_HIP_YAW = joint(
    "l_leg_hpz"
)

L_HIP_ROLL = joint(
    "l_leg_hpx"
)

L_HIP_PITCH = joint(
    "l_leg_hpy"
)

L_KNEE = joint(
    "l_leg_kny"
)

L_ANKLE_PITCH = joint(
    "l_leg_aky"
)

L_ANKLE_ROLL = joint(
    "l_leg_akx"
)


R_HIP_YAW = joint(
    "r_leg_hpz"
)

R_HIP_ROLL = joint(
    "r_leg_hpx"
)

R_HIP_PITCH = joint(
    "r_leg_hpy"
)

R_KNEE = joint(
    "r_leg_kny"
)

R_ANKLE_PITCH = joint(
    "r_leg_aky"
)

R_ANKLE_ROLL = joint(
    "r_leg_akx"
)


# ============================================================
# MOTOR
# ============================================================

def motor(
    jid,
    posicion,
    fuerza=FUERZA
):

    if jid is None:
        return

    p.setJointMotorControl2(

        atlas,

        jid,

        p.POSITION_CONTROL,

        targetPosition=posicion,

        force=fuerza,

        positionGain=KP,

        velocityGain=KD
    )


# ============================================================
# POSTURA ESTABLE
# ============================================================

def postura():

    # -------------------------------
    # CADERA
    # -------------------------------

    motor(
        L_HIP_YAW,
        0
    )

    motor(
        R_HIP_YAW,
        0
    )


    motor(
        L_HIP_ROLL,
        0
    )

    motor(
        R_HIP_ROLL,
        0
    )


    # -------------------------------
    # CADERA PITCH
    # -------------------------------

    motor(
        L_HIP_PITCH,
        -0.10
    )

    motor(
        R_HIP_PITCH,
        -0.10
    )


    # -------------------------------
    # RODILLA
    # -------------------------------

    motor(
        L_KNEE,
        0.22
    )

    motor(
        R_KNEE,
        0.22
    )


    # -------------------------------
    # TOBILLO
    # -------------------------------

    motor(
        L_ANKLE_PITCH,
        -0.10
    )

    motor(
        R_ANKLE_PITCH,
        -0.10
    )


    motor(
        L_ANKLE_ROLL,
        0
    )

    motor(
        R_ANKLE_ROLL,
        0
    )


# ============================================================
# ORIENTACIÓN
# ============================================================

def estado():

    posicion, quaternion = (
        p.getBasePositionAndOrientation(
            atlas
        )
    )

    roll, pitch, yaw = (
        p.getEulerFromQuaternion(
            quaternion
        )
    )

    velocidad_lineal, velocidad_angular = (
        p.getBaseVelocity(
            atlas
        )
    )

    return (
        posicion,
        roll,
        pitch,
        yaw,
        velocidad_angular
    )


# ============================================================
# EQUILIBRIO
# ============================================================

def equilibrio():

    (
        posicion,
        roll,
        pitch,
        yaw,
        velocidad

    ) = estado()


    velocidad_roll = velocidad[0]

    velocidad_pitch = velocidad[1]


    # ========================================================
    # PITCH
    # ========================================================

    pitch_corr = (

        -BALANCE_KP * pitch

        - BALANCE_KD
        * velocidad_pitch

    )


    # ========================================================
    # ROLL
    # ========================================================

    roll_corr = (

        -BALANCE_KP * roll

        - BALANCE_KD
        * velocidad_roll

    )


    # ========================================================
    # LIMITAR
    # ========================================================

    pitch_corr = max(
        -0.08,
        min(
            0.08,
            pitch_corr
        )
    )


    roll_corr = max(
        -0.06,
        min(
            0.06,
            roll_corr
        )
    )


    # ========================================================
    # APLICAR
    # ========================================================

    motor(
        L_HIP_PITCH,
        -0.10 + pitch_corr
    )

    motor(
        R_HIP_PITCH,
        -0.10 + pitch_corr
    )


    motor(
        L_ANKLE_PITCH,
        -0.10 - pitch_corr
    )

    motor(
        R_ANKLE_PITCH,
        -0.10 - pitch_corr
    )


    motor(
        L_HIP_ROLL,
        roll_corr
    )

    motor(
        R_HIP_ROLL,
        roll_corr
    )


    motor(
        L_ANKLE_ROLL,
        -roll_corr
    )

    motor(
        R_ANKLE_ROLL,
        -roll_corr
    )


# ============================================================
# CONTROLES
# ============================================================

print()
print("========================================")
print("           ATLAS ESTABILIDAD")
print("========================================")
print()
print("Q = salir")
print()
print("No caminar todavía.")
print("Primero estabilizamos Atlas.")
print()
print("========================================")
print()


# ============================================================
# POSTURA INICIAL
# ============================================================

for i in range(480):

    postura()

    p.stepSimulation()

    time.sleep(DT)


print(
    "Control de equilibrio activado."
)


# ============================================================
# BUCLE
# ============================================================

try:

    while True:

        eventos = p.getKeyboardEvents()


        # ----------------------------------------------------
        # SALIR
        # ----------------------------------------------------

        salir = False

        if ord("q") in eventos:

            if eventos[
                ord("q")
            ] & p.KEY_WAS_TRIGGERED:

                salir = True


        if salir:

            break


        # ----------------------------------------------------
        # EQUILIBRIO
        # ----------------------------------------------------

        equilibrio()


        # ----------------------------------------------------
        # SIMULACIÓN
        # ----------------------------------------------------

        p.stepSimulation()

        time.sleep(DT)


except KeyboardInterrupt:

    pass


finally:

    if p.isConnected():

        p.disconnect()
