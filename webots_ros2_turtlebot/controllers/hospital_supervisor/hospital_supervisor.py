"""Hospital supervisor controller.

Bei jedem Durchlauf wird der physische Zustand jedes Patienten zufaellig
erzeugt: im Bett liegend, auf dem Boden liegend (kollabiert) oder stehend.

Setze die Umgebungsvariable ARAMS_PATIENT_SEED, um einen Durchlauf
reproduzierbar zu machen (z.B. fuer Tests).
"""

import os
import random

from controller import Supervisor

# Patient-DEF-Name -> (x, y) des zugehoerigen Bettes.
# Alle Betten haben dieselbe Ausrichtung (rotation 0 0 1 1.5708),
# daher koennen die Posen relativ zur Bettposition berechnet werden.
PATIENT_BEDS = {
    'PATIENT_1': (-6.5, 6.0),
    'PATIENT_2': (0.0, 6.5),
    'PATIENT_3': (6.5, 6.0),
}

# Standhoehe des Pedestrian-Modells (siehe pedestrian.py ROOT_HEIGHT).
STANDING_HEIGHT = 1.27

STATES = ('in_bed', 'on_floor', 'standing')


def pose_for(state, bed_x, bed_y):
    """Liefert (translation, rotation) fuer einen Patientenzustand."""
    if state == 'in_bed':
        # Auf der Matratze liegend (entspricht der urspruenglich platzierten Pose).
        return [bed_x - 0.41, bed_y, 0.8], [0.0, 1.0, 0.0, -1.5708]
    if state == 'on_floor':
        # Kollabiert auf dem Boden, direkt vor dem Bett (Richtung Rauminneres).
        return [bed_x, bed_y - 1.0, 0.15], [-0.4377, 0.8629, -0.2527, -1.7178]
    # standing: aufrecht, einen Schritt vor dem Bett.
    return [bed_x, bed_y - 1.3, STANDING_HEIGHT], [0.0, 0.0, 1.0, 0.0]


def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    seed = os.environ.get('ARAMS_PATIENT_SEED')
    rng = random.Random(int(seed)) if seed is not None else random.Random()
    if seed is not None:
        print('[hospital_supervisor] Verwende festen Seed: {}'.format(seed))

    print('[hospital_supervisor] Patientenzustaende fuer diesen Durchlauf:')
    for def_name, (bed_x, bed_y) in PATIENT_BEDS.items():
        node = robot.getFromDef(def_name)
        if node is None:
            print('[hospital_supervisor] WARNUNG: Knoten {} nicht gefunden'.format(def_name))
            continue

        state = rng.choice(STATES)
        translation, rotation = pose_for(state, bed_x, bed_y)
        node.getField('translation').setSFVec3f(translation)
        node.getField('rotation').setSFRotation(rotation)
        node.resetPhysics()
        print('  {}: {}'.format(def_name, state))

    # Controller am Leben halten.
    while robot.step(timestep) != -1:
        pass


if __name__ == '__main__':
    main()
