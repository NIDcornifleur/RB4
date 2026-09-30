#!/usr/bin/env python3
# ================================================================
#  RBBB - Balayage de reglages (sur le PC), mesure du balancement lent
#     python balayage.py
#  Pour chaque jeu : envoi en direct (non enregistre), 8 s de stabilisation,
#  30 s de mesure (journal 50/s), puis :
#    - ecart-type de l'inclinaison dans la bande 0,3-3 Hz (balancement)
#    - frequence du pic, ecart-type de la vitesse des roues
#  Part des reglages ACTIFS dans le MM1 (pas des reglages enregistres :
#  l'angle essaye en direct peut differer) et les remet a la fin, ou si
#  le robot tombe.
#  Essais dans un fichier JSON, sinon la liste ESSAIS :
#     python balayage.py essais.json
#     (contenu : [["ref", {}], ["KV 0.001", {"v": 0.001}]])
# ================================================================
import json
import sys
import time
import urllib.request

import numpy as np

U = "http://192.168.2.226:8000"
STAB, MESURE = 8.0, 30.0
ESSAIS = [
    ("KP2600 KD900 KV0.0007",   {"p": 2600, "d": 900, "v": 0.0007}),
]


def get(chemin):
    with urllib.request.urlopen(U + chemin, timeout=5) as r:
        return json.load(r)


def envoyer(r):
    req = urllib.request.Request(U + "/reglages", data=json.dumps(r).encode(),
                                 headers={"Content-Type": "application/json"})
    urllib.request.urlopen(req, timeout=5).read()


def mesurer(debut):
    L = [l for l in get("/journal?depuis=%f" % debut)["lignes"] if l[1] is not None]
    t = np.array([l[1] for l in L]) / 1000.0
    i = np.array([l[2] for l in L]); v = np.array([l[4] for l in L])
    fe = (len(t) - 1) / (t[-1] - t[0])
    # inclinaison limitee a la bande 0,3-3 Hz (filtre par FFT), puis ecart-type
    A = np.fft.rfft(i - i.mean()); f = np.fft.rfftfreq(len(i), 1 / fe)
    bande = (f >= 0.3) & (f <= 3.0)
    std_bande = np.fft.irfft(A * bande, len(i)).std()
    fpic = f[bande][np.argmax(np.abs(A[bande]))]
    # vibration rapide : vitesse des roues commandee au-dessus de 5 Hz
    V = np.fft.rfft(v - v.mean())
    v_rapide = np.fft.irfft(V * (f >= 5.0), len(v)).std()
    # promenade lente : vitesse des roues sous 0,3 Hz
    v_lent = np.fft.irfft(V * (f < 0.3), len(v)).std()
    return std_bande, fpic, v_rapide, v_lent, all(l[5] for l in L)


if len(sys.argv) > 3:          # durees optionnelles : stabilisation, mesure (s)
    STAB, MESURE = float(sys.argv[2]), float(sys.argv[3])
if len(sys.argv) > 1:
    # fichier JSON (PowerShell 5.1 abime les guillemets sur la ligne de commande)
    with open(sys.argv[1], encoding="utf-8") as f:
        ESSAIS = [(nom, ch) for nom, ch in json.load(f)]
actifs = get("/etat")["reglages_mm1"]
base = {k: actifs[k] for k in ("p", "d", "v", "a", "f", "i", "e", "r", "b") if k in actifs}
print("Reglages actifs du MM1 (point de depart) : %s" % base)
resultats = []
try:
    for nom, changement in ESSAIS:
        r = dict(base); r.update(changement)
        envoyer(r)
        time.sleep(STAB)
        debut = time.time()
        fin = debut + MESURE
        while time.time() < fin:
            time.sleep(1)
            if not get("/etat")["actif"]:
                raise RuntimeError("robot tombe pendant l'essai %s" % nom)
        sb, fp, si, sv, tout_actif = mesurer(debut)
        resultats.append((nom, sb, fp, si, sv))
        print("%-24s balancement %.3f deg (pic %.2f Hz) | bruit roues > 5 Hz %5.1f pas/s | promenade < 0,3 Hz %4.0f pas/s%s"
              % (nom, sb, fp, si, sv, "" if tout_actif else "  (inactif par moments)"), flush=True)
except Exception as e:
    print("*** %s" % e)
finally:
    envoyer(base)
    print("Reglages de depart remis : %s" % base)



