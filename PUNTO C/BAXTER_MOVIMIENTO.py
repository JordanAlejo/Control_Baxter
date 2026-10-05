import pybullet as p
import pybullet_data
import time
import serial
import math


# ============================================================
# CONFIGURACIÓN
# ============================================================

PUERTO_ESP32 = "COM5"       # <-- CAMBIA A TU COM
BAUDRATE = 115200

DT = 1 / 240

# ============================================================
# VELOCIDADES
# ============================================================

# Movimiento MUY controlado
VELOCIDAD_MAX = 0.0030

# Rotación
VELOCIDAD_ROT = 0.008

# Aceleración
ACELERACION = 0.00006

# Frenado
FRENADO = 0.00010


# ============================================================
# CONEXIÓN ESP32
# ============================================================

try:

    esp32 = serial.Serial(
        PUERTO_ESP32,
        BAUDRATE,
        timeout=0.001
    )

    time.sleep(2)

    print("======================================")
    print(" ESP32 CONECTADO")
    print(" Puerto:", PUERTO_ESP32)
    print("======================================")

except Exception as e:

    print("======================================")
    print(" ESP32 NO CONECTADO")
    print(" Se usará el teclado del PC")
    print("======================================")

    esp32 = None


# ============================================================
# PYBULLET
# ============================================================

physics_client = p.connect(p.GUI)

p.setAdditionalSearchPath(
    pybullet_data.getDataPath()
)

p.setGravity(
    0,
    0,
    -9.81
)

p.setTimeStep(DT)


# ============================================================
# PISO
# ============================================================

plane = p.loadURDF(
    "plane.urdf",
    [0, 0, 0],
    useFixedBase=True
)


# ============================================================
# BAXTER
# ============================================================

BAXTER_URDF = (
    "baxter_common/"
    "baxter_description/"
    "urdf/"
    "toms_baxter.urdf"
)

baxter = p.loadURDF(
    BAXTER_URDF,
    [0, 0, 0],
    p.getQuaternionFromEuler(
        [0, 0, 0]
    ),
    useFixedBase=True
)


print()
print("======================================")
print("          BAXTER CARGADO")
print("======================================")


# ============================================================
# POSICIÓN
# ============================================================

posicion = [
    0.0,
    0.0,
    0.0
]

angulo = 0.0


# ============================================================
# VELOCIDADES ACTUALES
# ============================================================

vx = 0.0
vy = 0.0
vel_rot = 0.0


# ============================================================
# VELOCIDADES DESEADAS
# ============================================================

objetivo_vx = 0.0
objetivo_vy = 0.0
objetivo_rot = 0.0


# ============================================================
# FUNCIÓN DE ACELERACIÓN SUAVE
# ============================================================

def aproximar(valor, objetivo, paso):

    if valor < objetivo:

        valor += paso

        if valor > objetivo:
            valor = objetivo

    elif valor > objetivo:

        valor -= paso

        if valor < objetivo:
            valor = objetivo

    return valor


# ============================================================
# ACTUALIZAR VELOCIDADES
# ============================================================

def actualizar_velocidades():

    global vx
    global vy
    global vel_rot

    vx = aproximar(
        vx,
        objetivo_vx,
        ACELERACION
    )

    vy = aproximar(
        vy,
        objetivo_vy,
        ACELERACION
    )

    vel_rot = aproximar(
        vel_rot,
        objetivo_rot,
        ACELERACION * 4
    )


# ============================================================
# ACTUALIZAR POSICIÓN
# ============================================================

def actualizar_baxter():

    global posicion
    global angulo

    # --------------------------------------------------------
    # ROTACIÓN
    # --------------------------------------------------------

    angulo += vel_rot


    # --------------------------------------------------------
    # MOVIMIENTO LOCAL → MOVIMIENTO GLOBAL
    # --------------------------------------------------------

    dx = (
        vx * math.cos(angulo)
        -
        vy * math.sin(angulo)
    )

    dy = (
        vx * math.sin(angulo)
        +
        vy * math.cos(angulo)
    )


    posicion[0] += dx
    posicion[1] += dy


    # --------------------------------------------------------
    # ORIENTACIÓN
    # --------------------------------------------------------

    orientacion = p.getQuaternionFromEuler(
        [
            0,
            0,
            angulo
        ]
    )


    # --------------------------------------------------------
    # ACTUALIZAR BAXTER
    # --------------------------------------------------------

    p.resetBasePositionAndOrientation(
        baxter,
        posicion,
        orientacion
    )


# ============================================================
# COMANDOS ESP32
# ============================================================

def procesar_comando(comando):

    global objetivo_vx
    global objetivo_vy
    global objetivo_rot

    comando = comando.strip().upper()


    # --------------------------------------------------------
    # ADELANTE
    # --------------------------------------------------------

    if comando == "ADELANTE":

        objetivo_vx = VELOCIDAD_MAX
        objetivo_vy = 0
        objetivo_rot = 0


    # --------------------------------------------------------
    # ATRÁS
    # --------------------------------------------------------

    elif comando == "ATRAS":

        objetivo_vx = -VELOCIDAD_MAX
        objetivo_vy = 0
        objetivo_rot = 0


    # --------------------------------------------------------
    # IZQUIERDA
    # --------------------------------------------------------

    elif comando == "IZQUIERDA":

        objetivo_vx = 0
        objetivo_vy = VELOCIDAD_MAX
        objetivo_rot = 0


    # --------------------------------------------------------
    # DERECHA
    # --------------------------------------------------------

    elif comando == "DERECHA":

        objetivo_vx = 0
        objetivo_vy = -VELOCIDAD_MAX
        objetivo_rot = 0


    # --------------------------------------------------------
    # ROTAR IZQUIERDA
    # --------------------------------------------------------

    elif comando == "ROTAR_IZQ":

        objetivo_vx = 0
        objetivo_vy = 0
        objetivo_rot = VELOCIDAD_ROT


    # --------------------------------------------------------
    # ROTAR DERECHA
    # --------------------------------------------------------

    elif comando == "ROTAR_DER":

        objetivo_vx = 0
        objetivo_vy = 0
        objetivo_rot = -VELOCIDAD_ROT


    # --------------------------------------------------------
    # DETENER
    # --------------------------------------------------------

    elif comando == "DETENER":

        objetivo_vx = 0
        objetivo_vy = 0
        objetivo_rot = 0


# ============================================================
# TECLADO DEL PC
# ============================================================

def leer_teclado_pc():

    global objetivo_vx
    global objetivo_vy
    global objetivo_rot

    teclas = p.getKeyboardEvents()


    # ========================================================
    # ESTADO DE LAS DIRECCIONES
    # ========================================================

    adelante = False
    atras = False
    izquierda = False
    derecha = False

    rotar_izq = False
    rotar_der = False


    # ========================================================
    # FLECHA ARRIBA
    # ========================================================

    if p.B3G_UP_ARROW in teclas:

        if teclas[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:

            adelante = True


    # ========================================================
    # FLECHA ABAJO
    # ========================================================

    if p.B3G_DOWN_ARROW in teclas:

        if teclas[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:

            atras = True


    # ========================================================
    # FLECHA IZQUIERDA
    # ========================================================

    if p.B3G_LEFT_ARROW in teclas:

        if teclas[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:

            izquierda = True


    # ========================================================
    # FLECHA DERECHA
    # ========================================================

    if p.B3G_RIGHT_ARROW in teclas:

        if teclas[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:

            derecha = True


    # ========================================================
    # L
    # ========================================================

    if ord('l') in teclas:

        if teclas[ord('l')] & p.KEY_IS_DOWN:

            rotar_izq = True


    # ========================================================
    # K
    # ========================================================

    if ord('k') in teclas:

        if teclas[ord('k')] & p.KEY_IS_DOWN:

            rotar_der = True


    # ========================================================
    # PRIORIDAD DE ROTACIÓN
    # ========================================================

    if rotar_izq:

        objetivo_vx = 0
        objetivo_vy = 0
        objetivo_rot = VELOCIDAD_ROT

    elif rotar_der:

        objetivo_vx = 0
        objetivo_vy = 0
        objetivo_rot = -VELOCIDAD_ROT

    # ========================================================
    # MOVIMIENTO
    # ========================================================

    elif adelante:

        objetivo_vx = VELOCIDAD_MAX
        objetivo_vy = 0
        objetivo_rot = 0

    elif atras:

        objetivo_vx = -VELOCIDAD_MAX
        objetivo_vy = 0
        objetivo_rot = 0

    elif izquierda:

        objetivo_vx = 0
        objetivo_vy = VELOCIDAD_MAX
        objetivo_rot = 0

    elif derecha:

        objetivo_vx = 0
        objetivo_vy = -VELOCIDAD_MAX
        objetivo_rot = 0

    else:

        # No se presiona nada
        # → frenar suavemente

        objetivo_vx = 0
        objetivo_vy = 0
        objetivo_rot = 0


    # ========================================================
    # SPACE
    # ========================================================

    if ord(' ') in teclas:

        if teclas[ord(' ')] & p.KEY_IS_DOWN:

            objetivo_vx = 0
            objetivo_vy = 0
            objetivo_rot = 0


    # ========================================================
    # Q
    # ========================================================

    if ord('q') in teclas:

        if teclas[ord('q')] & p.KEY_WAS_TRIGGERED:

            return False


    return True


# ============================================================
# INFORMACIÓN
# ============================================================

print()
print("======================================")
print("       CONTROL DEL BAXTER")
print("======================================")
print()
print("PC:")
print(" ↑ = Adelante")
print(" ↓ = Atrás")
print(" ← = Izquierda")
print(" → = Derecha")
print(" L = Rotar izquierda")
print(" K = Rotar derecha")
print(" SPACE = Parar")
print(" Q = Salir")
print()
print("ESP32:")
print(" 1 = Rotar izquierda")
print(" 2 = Adelante")
print(" 3 = Rotar derecha")
print(" 4 = Izquierda")
print(" 5 = Parar")
print(" 6 = Derecha")
print(" 8 = Atrás")
print(" 0 = Salir")
print()
print("======================================")


# ============================================================
# LOOP
# ============================================================

ejecutando = True


while ejecutando:

    # --------------------------------------------------------
    # TECLADO PC
    # --------------------------------------------------------

    ejecutando = leer_teclado_pc()


    # --------------------------------------------------------
    # ESP32
    # --------------------------------------------------------

    if esp32 is not None:

        try:

            while esp32.in_waiting > 0:

                datos = esp32.readline().decode(
                    "utf-8",
                    errors="ignore"
                ).strip()


                if datos in [
                    "ADELANTE",
                    "ATRAS",
                    "IZQUIERDA",
                    "DERECHA",
                    "ROTAR_IZQ",
                    "ROTAR_DER",
                    "DETENER"
                ]:

                    procesar_comando(datos)


                elif datos == "SALIR":

                    ejecutando = False


        except Exception as e:

            print(
                "Error ESP32:",
                e
            )


    # --------------------------------------------------------
    # ACELERACIÓN / FRENADO
    # --------------------------------------------------------

    actualizar_velocidades()


    # --------------------------------------------------------
    # MOVIMIENTO
    # --------------------------------------------------------

    actualizar_baxter()


    # --------------------------------------------------------
    # SIMULACIÓN
    # --------------------------------------------------------

    p.stepSimulation()

    time.sleep(DT)


# ============================================================
# FINAL
# ============================================================

if esp32 is not None:

    esp32.close()

p.disconnect()

print("Programa terminado.")