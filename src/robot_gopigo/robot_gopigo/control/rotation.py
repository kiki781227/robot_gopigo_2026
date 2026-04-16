from gopigo3 import GoPiGo3
import time

gpg = GoPiGo3()

left_start = gpg.get_motor_encoder(gpg.MOTOR_LEFT)
right_start = gpg.get_motor_encoder(gpg.MOTOR_RIGHT)
print("Départ :", left_start, right_start)

try:
    gpg.set_motor_dps(gpg.MOTOR_LEFT, -100)  # recule
    gpg.set_motor_dps(gpg.MOTOR_RIGHT, 100)  # avance
    time.sleep(6)  # ← ajuste ce chiffre
finally:
    gpg.set_motor_dps(gpg.MOTOR_LEFT, 0)
    gpg.set_motor_dps(gpg.MOTOR_RIGHT, 0)

left_end = gpg.get_motor_encoder(gpg.MOTOR_LEFT)
right_end = gpg.get_motor_encoder(gpg.MOTOR_RIGHT)

print("Ticks gauche :", left_end - left_start)
print("Ticks droite :", right_end - right_start)
print("Différence entre les deux roues :", abs((left_end - left_start) - (right_end - right_start)))
