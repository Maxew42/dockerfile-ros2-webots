"""Voiture autonome avec utilisation d'un
LIDAR sur WEBOTS
Auteur : Chrysanthe et Jessica
"""

import numpy as np
import random
import time

import sys
sys.path.append("../")
from car_logic import CarLogic
import json
# --------------GYM----------------------------

# Création de l'environnement GYM
class NaiveMinCarController(CarLogic):
    def __init__(self,path_to_trajectory=None):
        super().__init__( path_to_trajectory=path_to_trajectory)  # Objet héritant la classe Driver
    # Vérification de l'état de la voiture
    def observe(self):
        try:
            tableau = self.lidar.getRangeImage()
            # Division par 10 pour que la valeur soit entre 0 et 1
            etat = np.divide(np.array(tableau), 10)
        except:  # En cas de non retour lidar
            print("Pas de retour du lidar")
            etat = np.zeros(self.lidar.getNumberOfPoints())

        return np.array(etat).astype('float32')

    # Fonction pour detection de collision et attribution des récompenses
    def evaluer(self):
        recompense = 0
        done = False

        id_balise = self.get_balise()

        self.update_advancement(id_balise)
        self.update_travelled_distance()
        xy_lidar = list(map(lambda x: (x.x, x.y), self.lidar.getPointCloud()))
        y_lidar = list(map(lambda x: x.y, self.lidar.getPointCloud()))

        dist = list(map(lambda p: p[0] ** 2 + p[1] ** 2, xy_lidar))
        front_dist = np.array(dist[-8:] + dist[:8])
        min_front_dist = front_dist[np.isfinite(front_dist)].min()
        id = dist.index(min_front_dist)

        if id in [0, 1, len(dist), len(dist) - 1]:
            self.setSpeedCommand(-0.5)
            self.setSteeringCommand(-0.1)
        elif y_lidar[id] < 0:
            self.setSpeedCommand(0.5)
            self.setSteeringCommand(-0.4)
        else:
            self.setSpeedCommand(0.5)
            self.setSteeringCommand(0.4)

        etat = self.get_normalized_lidar_range_image()
        return recompense, done

    # Fonction step de l'environnement GYM
    def step(self, action):

        obs = self.observe()

        reward, done = self.evaluer()

        super().step()

        return obs, reward, done, {}

    # Fonction render de l'environnement GYM
    def render(self, mode="human", close=False):
        pass


# ----------------Programme principal--------------------
def main():
    path_to_trajectory = r"/home/worlds/trajectories/MultiTrackEasy_1--1303-1312.json"

    carController = NaiveMinCarController(path_to_trajectory =path_to_trajectory )
    
    t0 = time.time()
    print("Running")
    for i in range(2):
        carController.reset()
        for j in range(20):
            carController.step(random.randint(0, 4))
            print(f"etape {j}")
        print("Reseting")
    print("Fin du roulage :)")
    print(carController.full_logs)
    t1 = time.time()
    print(f"Time for simulation: {t1-t0:.3f}")
    with open(r"/home/ok.json", 'w', encoding='utf-8') as f:
        json.dump(carController.full_logs, f, ensure_ascii=False, indent=4)


if __name__ == '__main__':
    main()

