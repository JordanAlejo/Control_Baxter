import pybullet as p
import pybullet_data
import time
import math
import sys

# ============================================================
# CONFIGURACIÓN
# ============================================================

URDF = "atlas/atlas_v4_with_multisense.urdf"
DT = 1.0 / 240.0

HEADLESS = "--headless" in sys.argv

if HEADLESS:
    p.connect(p.DIRECT)
else:
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)
p.setTimeStep(DT)
p.setPhysicsEngineParameter(fixedTimeStep=DT, numSolverIterations=80, numSubSteps=1)

# ============================================================
# PISO
# ============================================================

plane = p.loadURDF("plane.urdf", [0, 0, 0])

# ============================================================
# ATLAS
# ============================================================

atlas = p.loadURDF(URDF, [0, 0, 1.15], useFixedBase=False)

# ============================================================
# CONFIGURACIÓN DE CONTROL
# ============================================================

FUERZA = 400
KP = 0.25
KD = 0.08

BALANCE_KP = 0.7
BALANCE_KD = 0.12

# Parámetros de la marcha
T_CICLO = 1.4          # segundos por ciclo completo
AMP_HIP = 0.35         # amplitud de pitch de cadera
AMP_KNEE = 0.65        # flexión de rodilla en fase de balanceo
AMP_ROLL = 0.10        # desplazamiento lateral del peso
PASO_VEL = 1.0         # multiplicador de velocidad de marcha

# ============================================================
# JOINTS
# ============================================================

joints = {}

for i in range(p.getNumJoints(atlas)):
    info = p.getJointInfo(atlas, i)
    nombre = info[1].decode("utf-8")
    joints[nombre] = i


def joint(nombre):
    return joints.get(nombre, None)


# ============================================================
# PIERNAS
# ============================================================

L_HIP_YAW = joint("l_leg_hpz")
L_HIP_ROLL = joint("l_leg_hpx")
L_HIP_PITCH = joint("l_leg_hpy")
L_KNEE = joint("l_leg_kny")
L_ANKLE_PITCH = joint("l_leg_aky")
L_ANKLE_ROLL = joint("l_leg_akx")

R_HIP_YAW = joint("r_leg_hpz")
R_HIP_ROLL = joint("r_leg_hpx")
R_HIP_PITCH = joint("r_leg_hpy")
R_KNEE = joint("r_leg_kny")
R_ANKLE_PITCH = joint("r_leg_aky")
R_ANKLE_ROLL = joint("r_leg_akx")

# ============================================================
# MOTOR
# ============================================================

def motor(jid, posicion, fuerza=FUERZA):
    if jid is None:
        return
    p.setJointMotorControl2(
        atlas,
        jid,
        p.POSITION_CONTROL,
        targetPosition=posicion,
        force=fuerza,
        positionGain=KP,
        velocityGain=KD,
    )


# ============================================================
# POSTURA ESTABLE
# ============================================================

def postura():
    motor(L_HIP_YAW, 0)
    motor(R_HIP_YAW, 0)
    motor(L_HIP_ROLL, 0)
    motor(R_HIP_ROLL, 0)
    motor(L_HIP_PITCH, -0.10)
    motor(R_HIP_PITCH, -0.10)
    motor(L_KNEE, 0.22)
    motor(R_KNEE, 0.22)
    motor(L_ANKLE_PITCH, -0.10)
    motor(R_ANKLE_PITCH, -0.10)
    motor(L_ANKLE_ROLL, 0)
    motor(R_ANKLE_ROLL, 0)


# ============================================================
# ORIENTACIÓN
# ============================================================

def estado():
    posicion, quaternion = p.getBasePositionAndOrientation(atlas)
    roll, pitch, yaw = p.getEulerFromQuaternion(quaternion)
    velocidad_lineal, velocidad_angular = p.getBaseVelocity(atlas)
    return (posicion, roll, pitch, yaw, velocidad_angular)


# ============================================================
# EQUILIBRIO (corrige sobre las metas actuales)
# ============================================================

def correccion_equilibrio():
    (posicion, roll, pitch, yaw, velocidad) = estado()

    velocidad_roll = velocidad[0]
    velocidad_pitch = velocidad[1]

    pitch_corr = -BALANCE_KP * pitch - BALANCE_KD * velocidad_pitch
    roll_corr = -BALANCE_KP * roll - BALANCE_KD * velocidad_roll

    pitch_corr = max(-0.08, min(0.08, pitch_corr))
    roll_corr = max(-0.06, min(0.06, roll_corr))

    return pitch_corr, roll_corr


# ============================================================
# MARCHA (caminar)
# ============================================================

def marcha(t, pitch_corr, roll_corr, amp_scale):
    """
    Genera los ángulos de cadera/rodilla/tobillo para caminar.
    amp_scale va de 0 a 1 para arrancar suave desde la postura.
    """
    fase = 2.0 * math.pi * t / T_CICLO

    # desplazamiento lateral del peso (de un pie al otro)
    peso = AMP_ROLL * math.sin(fase) * amp_scale

    # pierna izquierda adelante cuando sin(fase) > 0, derecha en caso contrario
    l_swing = max(0.0, math.sin(fase))
    r_swing = max(0.0, -math.sin(fase))

    # hip pitch: la pierna oscila adelante/atrás
    l_hip = AMP_HIP * math.sin(fase) * amp_scale - 0.10
    r_hip = -AMP_HIP * math.sin(fase) * amp_scale - 0.10

    # rodilla: se flexiona en fase de balanceo
    l_knee = 0.22 + AMP_KNEE * l_swing * amp_scale
    r_knee = 0.22 + AMP_KNEE * r_swing * amp_scale

    # tobillo: compensa para mantener el pie plano
    l_ankle = -(l_hip + l_knee) * 0.9 - 0.10 + 0.10
    r_ankle = -(r_hip + r_knee) * 0.9 - 0.10 + 0.10

    # -------- APLICAR + CORRECCIÓN DE EQUILIBRIO --------

    # yaw de cadera
    motor(L_HIP_YAW, 0)
    motor(R_HIP_YAW, 0)

    # roll de cadera (cambio de peso) + corrección
    motor(L_HIP_ROLL, peso + roll_corr)
    motor(R_HIP_ROLL, peso + roll_corr)

    # pitch de cadera + corrección
    motor(L_HIP_PITCH, l_hip + pitch_corr)
    motor(R_HIP_PITCH, r_hip + pitch_corr)

    # rodillas
    motor(L_KNEE, l_knee)
    motor(R_KNEE, r_knee)

    # tobillos (pitch y roll)
    motor(L_ANKLE_PITCH, l_ankle - pitch_corr)
    motor(R_ANKLE_PITCH, r_ankle - pitch_corr)

    motor(L_ANKLE_ROLL, -(peso + roll_corr))
    motor(R_ANKLE_ROLL, -(peso + roll_corr))


# ============================================================
# CONTROLES
# ============================================================

print()
print("========================================")
print("        ATLAS: CAMINAR + EQUILIBRIO")
print("========================================")
print()
print("ESPACIO = iniciar / detener marcha")
print("Q      = salir")
print()

# ============================================================
# POSTURA INICIAL
# ============================================================

for i in range(480):
    postura()
    p.stepSimulation()
    if not HEADLESS:
        time.sleep(DT)

print("Control de equilibrio listo. Pulsa ESPACIO para caminar.")

# ============================================================
# BUCLE
# ============================================================

cargando = False      # True = caminando
t = 0.0               # tiempo de simulación
blend = 0.0           # 0 -> postura, 1 -> marcha completa

try:
    while True:
        eventos = p.getKeyboardEvents()

        # -------- SALIR --------
        if ord("q") in eventos and (eventos[ord("q")] & p.KEY_WAS_TRIGGERED):
            break

        # -------- TOGGLE CAMINAR --------
        if ord(" ") in eventos and (eventos[ord(" ")] & p.KEY_WAS_TRIGGERED):
            cargando = not cargando
            print("Marcha:", "ACTIVADA" if cargando else "DETENIDA")

        # suavizar la transición postura <-> marcha
        if cargando:
            blend = min(1.0, blend + DT / 1.0)
        else:
            blend = max(0.0, blend - DT / 1.0)

        pitch_corr, roll_corr = correccion_equilibrio()

        if blend > 0.0:
            marcha(t, pitch_corr, roll_corr, blend)
            t += DT * PASO_VEL
        else:
            # solo equilibrio sobre postura
            postura()
            motor(L_HIP_PITCH, -0.10 + pitch_corr)
            motor(R_HIP_PITCH, -0.10 + pitch_corr)
            motor(L_ANKLE_PITCH, -0.10 - pitch_corr)
            motor(R_ANKLE_PITCH, -0.10 - pitch_corr)
            motor(L_HIP_ROLL, roll_corr)
            motor(R_HIP_ROLL, roll_corr)
            motor(L_ANKLE_ROLL, -roll_corr)
            motor(R_ANKLE_ROLL, -roll_corr)

        p.stepSimulation()

        if not HEADLESS:
            time.sleep(DT)

        # modo prueba sin GUI: corta a los 8 segundos
        if HEADLESS:
            sim_time = p.getPhysicsEngineParameters()["fixedTimeStep"] * p.getPhysicsEngineParameters()["numSolverIterations"]
            if t > 8.0:
                pos, _ = p.getBasePositionAndOrientation(atlas)
                print("Posicion base tras prueba:", pos)
                break

except KeyboardInterrupt:
    pass

finally:
    if p.isConnected():
        p.disconnect()
