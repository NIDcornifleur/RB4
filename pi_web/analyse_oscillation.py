#!/usr/bin/env python3
# ================================================================
#  RBBB - Analyse de l'oscillation (a lancer sur le Pi)
#     python3 analyse_oscillation.py [duree_s] [diametre_roue_mm]
#  Attend <duree_s> secondes (defaut 30), puis lit le journal des lignes
#  "E" du serveur web (http://127.0.0.1:8000/journal) et calcule :
#    - amplitude et frequence de l'oscillation d'inclinaison,
#    - vitesse des roues et vitesse de chute a la meme frequence,
#    - si l'oscillation est reguliere (cycle limite) ou irreguliere (bruit).
#  Necessite le MM1 v1.4 (50 mesures/s avec t, g et v).
#  Donnees brutes : /tmp/journal_oscillation.csv
# ================================================================

import json
import sys
import time
import urllib.request

import numpy as np

DUREE = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
DIAMETRE_ROUE_MM = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0

debut = time.time()
print("Mesure pendant %.0f s : robot debout, sans le toucher..." % DUREE, flush=True)
time.sleep(DUREE)
with urllib.request.urlopen("http://127.0.0.1:8000/journal?depuis=%f" % debut, timeout=5) as r:
    lignes = json.load(r)["lignes"]

lignes = [l for l in lignes if l[1] is not None and l[3] is not None]
if len(lignes) < 100:
    sys.exit("Trop peu de mesures detaillees (%d) : le MM1 est-il en v1.4 ?" % len(lignes))
if not all(l[5] for l in lignes):
    print("Attention : l'equilibre n'a pas ete actif tout le temps.")

a = np.array([[l[1] / 1000.0, l[2], l[3], l[4]] for l in lignes], dtype=float)
t, i, g, v = a[:, 0] - a[0, 0], a[:, 1], a[:, 2], a[:, 3]
np.savetxt("/tmp/journal_oscillation.csv", np.c_[t, i, g, v], delimiter=",",
           header="t,inclinaison,vitesse_chute,vitesse_roues", comments="", fmt="%.3f")

fe = (len(t) - 1) / (t[-1] - t[0])
tu = np.arange(0, t[-1], 1 / fe)             # grille reguliere
I = np.interp(tu, t, i); G = np.interp(tu, t, g); V = np.interp(tu, t, v)


def spectre(x):
    x = (x - x.mean()) * np.hanning(len(x))
    m = 8 * len(x)                           # bourrage de zeros : frequences plus fines
    A = np.fft.rfft(x, m) * 4 / len(x)       # amplitude (fenetre de Hann)
    return np.fft.rfftfreq(m, 1 / fe), A


f, AI = spectre(I)
_, AG = spectre(G)
_, AV = spectre(V)
ok = f > 0.1
k = np.argmax(np.abs(AI) * ok)
f0 = f[k]

print("\nMesures : %d en %.1f s (%.0f par seconde)" % (len(t), t[-1], fe))
print("\nINCLINAISON : moyenne %.2f deg, ecart-type %.2f deg, crete a crete %.2f deg"
      % (I.mean(), I.std(), I.max() - I.min()))
print("Frequence dominante : %.2f Hz, amplitude %.2f deg" % (f0, abs(AI[k])))
M = np.abs(AI) * ok
sommets = [j for j in range(1, len(M) - 1) if M[j] >= M[j - 1] and M[j] > M[j + 1]]
sommets.sort(key=lambda j: -M[j])
print("Pics :", ", ".join("%.2f Hz (%.2f deg)" % (f[j], M[j]) for j in sommets[:5]))
hf = (f > 3)
print("Energie au-dessus de 3 Hz : %.0f %% de l'oscillation"
      % (100 * np.sum(np.abs(AI[hf]) ** 2) / np.sum(np.abs(AI[ok]) ** 2)))

print("\nVITESSE DES ROUES : moyenne %.0f pas/s, ecart-type %.0f pas/s"
      % (V.mean(), V.std()))
pas = abs(AV[k]) / (2 * np.pi * f0)          # amplitude du va-et-vient, en pas
tours = 2 * pas / 1600                          # aller-retour complet, en tours de roue
print("A %.2f Hz : %.0f pas/s, soit un va-et-vient de %.3f tour de roue"
      % (f0, abs(AV[k]), tours))
print("   (%.1f cm avec des roues de %.0f mm de diametre)"
      % (tours * np.pi * DIAMETRE_ROUE_MM / 10, DIAMETRE_ROUE_MM))
print("Dephasage roues / inclinaison : %.0f deg" % np.degrees(np.angle(AV[k] / AI[k])))

print("\nVITESSE DE CHUTE : ecart-type %.1f deg/s ; a %.2f Hz : %.1f deg/s"
      % (G.std(), f0, abs(AG[k])))
print("Controle : 2*pi*f*amplitude = %.1f deg/s (doit etre proche)"
      % (2 * np.pi * f0 * abs(AI[k])))

# Regularite : amplitude mesuree par tranches de 2 periodes
n = max(int(2 * fe / f0), 10)
tranches = [I[j:j + n].max() - I[j:j + n].min() for j in range(0, len(I) - n, n)]
cv = np.std(tranches) / np.mean(tranches)
print("\nREGULARITE : crete a crete par tranches de %.1f s : %s"
      % (n / fe, " ".join("%.2f" % x for x in tranches)))
if cv < 0.2:
    print("-> Oscillation tres reguliere (variation %.0f %%) : cycle limite," % (100 * cv))
    print("   cause probable : jeu mecanique, frottement ou non-linearite.")
else:
    print("-> Oscillation irreguliere (variation %.0f %%) : mode peu amorti" % (100 * cv))
    print("   excite par le bruit ; un meilleur reglage peut l'amortir.")
print("\nDonnees brutes : /tmp/journal_oscillation.csv")
