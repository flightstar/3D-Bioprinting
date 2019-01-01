# Application projet robotique : Bras SCARA ( R T R )

import RPi.GPIO as GPIO
from robot import *
import queue
import multiprocessing

debug = True

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
bras = Robot(a, b, debug)


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


def reset_callback(channel):
    """
    Called by interruption when the reset_bouton is pressed.
    """
    command_queue.put("reset")


"""Liaison boutons -> evenements"""
# GPIO.add_event_detect(bouton1, GPIO.RISING, callback = action1, bouncetime = 1000)
# GPIO.add_event_detect(bouton2, GPIO.RISING, callback = action2, bouncetime = 1000)
# GPIO.add_event_detect(bouton3, GPIO.RISING, callback = action3, bouncetime = 1000)
GPIO.add_event_detect(bouton_reset, GPIO.RISING, callback=reset_callback, bouncetime=1000)


def read_inputs(command_q):
    if GPIO.input(bouton1):
        command_q.put("action1")
    elif GPIO.input(bouton3):
        command_q.put("action3")

if __name__ == '__main__':
    main_process = multiprocessing.Process()
    command_queue = multiprocessing.Queue()
    print("Pret. ")
    try:
        while True:
            try:
                read_inputs(command_queue)
                command = command_queue.get(block=True, timeout=1)  # Check if there is a command to execute
                if command == "reset":
                    if debug:
                        print("Terminating Main_Process")
                    main_process.terminate()
                    main_process.join()
                    ranger_crayon()
                    bras.reset_position()
                elif command == "action1":
                    if debug:
                        print("Starting action1")
                    main_process = multiprocessing.Process(target=action1)
                    main_process.start()
                elif command == "action3":
                    if debug:
                        print("Starting action3")
                    main_process = multiprocessing.Process(target=action3)
                    main_process.start()
            except queue.Empty:
                time.sleep(0)
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        GPIO.cleanup()
        main_process.terminate()
        main_process.join()
