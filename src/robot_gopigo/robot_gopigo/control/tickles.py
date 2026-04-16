from gopigo3 import GoPiGo3

gpg = GoPiGo3()

# Encodeurs au départ
left_start = gpg.get_motor_encoder(gpg.MOTOR_LEFT)
right_start = gpg.get_motor_encoder(gpg.MOTOR_RIGHT)

print("Départ :", left_start, right_start)

try:
    # Rotation sur place
    # gauche avance, droite recule
    gpg.set_motor_dps(gpg.MOTOR_LEFT, 80)
    gpg.set_motor_dps(gpg.MOTOR_RIGHT, -80)

    input("➡️ Appuie sur ENTER quand le robot a fait EXACTEMENT 360°")

finally:
    # Stop moteurs
    gpg.set_motor_dps(gpg.MOTOR_LEFT, 0)
    gpg.set_motor_dps(gpg.MOTOR_RIGHT, 0)

# Encodeurs finaux
left_end = gpg.get_motor_encoder(gpg.MOTOR_LEFT)
right_end = gpg.get_motor_encoder(gpg.MOTOR_RIGHT)

# Calcul des ticks
ticks_left = left_end - left_start
ticks_right = right_end - right_start

# Rotation = différence entre les deux roues
ticks_rotation = abs(ticks_right - ticks_left)

print("\n--- Résultats rotation ---")
print("Ticks gauche :", ticks_left)
print("Ticks droite :", ticks_right)
print("Différence (360°) :", ticks_rotation)
