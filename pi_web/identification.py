#!/usr/bin/env python3
# ================================================================
#  RBBB - Identification du pendule a partir du journal (MM1 >= v1.4)
#     python3 identification.py [duree_s] [diametre_roue_mm]
#  Modele linearise d'un pendule inverse dont on impose l'acceleration
#  des roues (moteurs pas-a-pas) :
#     acc_angulaire = (g/L) * inclinaison  -  (1/L) * acc_roues + biais
#  On mesure inclinaison (i), vitesse de chute (g) et vitesse des roues (v),
#  on derive, puis on trouve g/L et 1/L par moindres carres.
#  Ecrit /tmp/identification.csv (donnees filtrees) pour la simulation.
# ================================================================
import json
import sys
import time
import urllib.request

import numpy as np

DUREE = float(sys.argv[1]) if len(sys.argv) > 1 else 50.0
D_ROUE = (float(sys.argv[2]) if len(sys.argv) > 2 else 82.0) / 1000.0
M_PAR_PAS = np.pi * D_ROUE / 1600.0       # 1600 pas par tour
G = 9.81

debut = time.time()
print("Mesure pendant %.0f s : robot debout, sans le toucher..." % DUREE, flush=True)
time.sleep(DUREE)
with urllib.request.urlopen("http://127.0.0.1:8000/journal?depuis=%f" % debut, timeout=5) as r:
    L = [l for l in json.load(r)["lignes"] if l[1] is not None and l[5]]
a = np.array([[l[1] / 1000.0, l[2], l[3], l[4]] for l in L])
t = a[:, 0] - a[0, 0]
th = np.radians(a[:, 1]); om = np.radians(a[:, 2]); v = a[:, 3] * M_PAR_PAS
fe = (len(t) - 1) / t[-1]
print("Mesures : %d a %.0f/s" % (len(t), fe))


def passe_bas(x, fc):
    # filtre sans dephasage (aller-retour) du 2e ordre
    k = 1 - np.exp(-2 * np.pi * fc / fe)
    for _ in range(2):
        y = np.empty_like(x); y[0] = x[0]
        for n in range(1, len(x)):
            y[n] = y[n - 1] + k * (x[n] - y[n - 1])
        x = y[::-1]
    return x


def derivee(x):
    return np.gradient(x, t)


res = []
for fc in (3.0, 5.0):
    thf = passe_bas(th, fc); omf = passe_bas(om, fc); vf = passe_bas(v, fc)
    alpha = derivee(omf)              # acceleration angulaire (rad/s2)
    acc = derivee(vf)                 # acceleration des roues (m/s2)
    c = slice(50, -50)                # bords du filtre ecartes
    X = np.c_[thf[c], acc[c], np.ones(len(thf[c]))]
    coef, *_ = np.linalg.lstsq(X, alpha[c], rcond=None)
    pred = X @ coef
    r2 = 1 - np.var(alpha[c] - pred) / np.var(alpha[c])
    gL, mUnL, biais = coef
    res.append((fc, gL, -mUnL, r2))
    print("\nFiltre %.0f Hz : g/L = %.1f /s2, 1/L = %.2f /m, ajustement R2 = %.2f"
          % (fc, gL, -mUnL, r2))
    if -mUnL > 0:
        print("  -> L d'apres 1/L : %.0f cm ; L d'apres g/L : %.0f cm"
              % (100 / -mUnL, 100 * G / gL if gL > 0 else float("nan")))
    # coherence : l'integrale de g doit suivre i
    print("  controle vitesse de chute : correlation derivee(i) / g = %.2f"
          % np.corrcoef(derivee(thf)[c], omf[c])[0, 1])

np.savetxt("/tmp/identification.csv", np.c_[t, th, om, v], delimiter=",",
           header="t,theta_rad,omega_rad_s,v_m_s", comments="", fmt="%.5f")
print("\nDonnees : /tmp/identification.csv")
