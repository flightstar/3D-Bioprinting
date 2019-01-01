# Application projet robotique : Bras SCARA ( R T R )

import RPi.GPIO as GPIO
from robot import *

# Initialisation
# Espace travail
bouton1 = 6
bouton2 = 13
bouton3 = 19
bouton_reset = 26
status_led_r = 0
status_led_g = 0
status_led_b = 0
GPIO.setmode(GPIO.BCM)

# Robot
a = 6.5  # longueur joint 1
b = 4.0  # longueur joint 2

# boutons
GPIO.setup(bouton1, GPIO.IN)
GPIO.setup(bouton2, GPIO.IN)
GPIO.setup(bouton3, GPIO.IN)
GPIO.setup(bouton_reset, GPIO.IN)
GPIO.setup(status_led_r, GPIO.OUT)
GPIO.setup(status_led_g, GPIO.OUT)
GPIO.setup(status_led_b, GPIO.OUT)

posCrayon = Point(a + b, 0, 4)

# Init.
bras = Robot(a, b, True)


def prendre_crayon():
    print(" Routine chercher crayon déclenchée")
    bras.go_to_coordinates(posCrayon.x, posCrayon.y, bras.MAX_HEIGHT)
    time.sleep(1)
    bras.open_gripper()
    bras.go_to_point(posCrayon)
    time.sleep(3)
    bras.close_gripper()
    time.sleep(2)
    bras.move_vertical(bras.MAX_HEIGHT)
    time.sleep(4)


def ranger_crayon():
    print(" Routine ranger crayon déclenché")
    bras.move_vertical(bras.MAX_HEIGHT)
    time.sleep(3)
    bras.go_to_coordinates(posCrayon.x, posCrayon.y, bras.MAX_HEIGHT)
    time.sleep(1.5)
    bras.go_to_point(posCrayon)
    time.sleep(1)
    bras.open_gripper()
    bras.move_vertical(bras.MAX_HEIGHT)


# Routines pour chaque bouton
def action1():
    print("Tracer ligne droite")
    start = Point(3, 9, 3.7)
    stop = Point(-3, 9, 3.7)

    prendre_crayon()
    print(" Crayon saisi, se placer pour dessin")
    # Se placer pour dessiner (hauteurMax)
    bras.go_to_coordinates(start.x, start.y, bras.MAX_HEIGHT)
    time.sleep(1)

    print(" Debut dessin")
    bras.go_to_point(start)
    time.sleep(3)

    bras.draw_line(start, stop)

    print(" Dessin termine.")
    ranger_crayon()


def action2():
    A = Point(3, 10, 3.7)
    B = Point(1, 10, 3.7)
    C = Point(1, 8, 3.7)
    D = Point(3, 8, 3.7)
    prendre_crayon()
    bras.go_to_coordinates(A.x, A.y, bras.MAX_HEIGHT)
    time.sleep(1)
    bras.go_to_point(A)
    time.sleep(3)
    print("  trace ligne A, B")
    bras.draw_line(A, B)
    time.sleep(3)
    print("  trace ligne B, C")
    bras.draw_line(B, C)
    time.sleep(3)
    print("  trace ligne C, D")
    bras.draw_line(C, D)
    time.sleep(3)
    print("  trace ligne D, A")
    bras.draw_line(D, A)
    time.sleep(3)

    print(" Dessin termine.")
    ranger_crayon()


def action3():
    print("Tracer triangle")
    A = Point(3.0, 9.0, 3.7)
    B = Point(-3.0, 9.0, 3.7)
    C = Point(-3.0, 8.0, 3.7)

    prendre_crayon()
    print(" Crayon saisi, se placer pour dessin")
    # Se placer pour dessiner (hauteurMax)
    bras.go_to_coordinates(A.x, A.y, bras.MAX_HEIGHT)
    time.sleep(1)

    print(" Debut dessin")
    bras.go_to_point(A)
    time.sleep(3)  # temps de descente

    bras.draw_line(A, B)
    print(" Point B atteint")
    time.sleep(0.5)
    bras.draw_line(B, C)
    print(" Point C atteint")
    time.sleep(0.5)
    bras.draw_line(C, A)

    print(" Dessin termine.")
    ranger_crayon()


def reinitialiser(channel):
    # print "bouton stop "
    ranger_crayon()
    bras.reset_position()


# Liaison boutons -> evenements
# GPIO.add_event_detect(bouton1, GPIO.RISING, callback = action1, bouncetime = 1000)
# GPIO.add_event_detect(bouton2, GPIO.RISING, callback = action2, bouncetime = 1000)
# GPIO.add_event_detect(bouton3, GPIO.RISING, callback = action3, bouncetime = 1000)
GPIO.add_event_detect(bouton_reset, GPIO.RISING, callback=reinitialiser, bouncetime=1000)

print(" Pret. ")

while True:
    if GPIO.input(bouton1):
        action1()
    if GPIO.input(bouton2):
        action2()
    if GPIO.input(bouton3):
        action3()
