#!/usr/bin/env python3
# ================================================================
#  RBBB - Essai d'excitation pour identifier le pendule (sur le PC)
#     python excitation.py [amplitude_deg] [demi_periode_s] [duree_s]
#  Alterne l'angle d'equilibre "a" de +/- amplitude autour de la valeur
#  enregistree, puis la remet. Rien n'est enregistre.
#  Ecrit excitation_journal.json (journal du MM1 + moments des changements).
# ================================================================
import json
import sys
import time
import urllib.request

U = "http://192.168.2.226:8000"
AMP = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
DEMI = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
DUREE = float(sys.argv[3]) if len(sys.argv) > 3 else 40.0


def get(chemin):
    with urllib.request.urlopen(U + chemin, timeout=5) as r:
        return json.load(r)


def reglage(a):
    req = urllib.request.Request(U + "/reglages", data=json.dumps({"a": a}).encode(),
                                 headers={"Content-Type": "application/json"})
    urllib.request.urlopen(req, timeout=5).read()


etat = get("/etat")
if not etat["actif"]:
    sys.exit("Robot non actif : essai annule")
a0 = etat["reglages_pi"]["a"]
changements = []
debut = time.time()
signe = 1
try:
    while time.time() - debut < DUREE:
        a = round(a0 + signe * AMP, 2)
        reglage(a)
        changements.append([time.time(), a])
        signe = -signe
        fin = time.time() + DEMI
        while time.time() < fin:
            time.sleep(0.2)
            if not get("/etat")["actif"]:
                raise RuntimeError("robot tombe")
finally:
    reglage(a0)
    changements.append([time.time(), a0])
    print("Angle remis a %.2f" % a0)

time.sleep(1)
j = get("/journal?depuis=%f" % (debut - 3))
with open(r"C:\RB4\modele\excitation_journal.json", "w") as f:
    json.dump({"a0": a0, "changements": changements, "journal": j["lignes"]}, f)
print("Lignes : %d, changements : %d" % (len(j["lignes"]), len(changements)))
