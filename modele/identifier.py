#!/usr/bin/env python3
# ================================================================
#  RBBB - Identification du pendule a partir de excitation_journal.json
#  Modele : acc_angulaire = (G/L)*th - acc_roues/L + biais
#  th = inclinaison par rapport a l'angle enregistre a0 :
#       i = a - angleX  ->  th = i - (a - a0)
#  Les changements de "a" se reperent aux sauts de i (angleX est continu).
# ================================================================
import json
import numpy as np

G = 9.81
C = np.pi * 0.082 / 1600

d = json.load(open(r"C:\RB4\modele\excitation_journal.json"))
a0 = d["a0"]
L_ = [l for l in d["journal"] if l[1] is not None]
t = np.array([l[1] for l in L_]) / 1000.0
i = np.array([l[2] for l in L_]); om = np.array([l[3] for l in L_]); v = np.array([l[4] for l in L_])
actif = np.array([l[5] for l in L_])
t -= t[0]
fe = (len(t) - 1) / t[-1]

# consignes successives (sans la derniere remise a a0, ajoutee a la fin)
consignes = [c[1] for c in d["changements"]]
# sauts de i : difference observee moins la variation attendue (om*dt)
di = np.diff(i) - om[1:] / fe
sauts = np.sort(np.argsort(-np.abs(di))[:len(consignes)] + 1)
a = np.full(len(i), a0)
for k, n in enumerate(sauts):
    a[n:] = consignes[k]
print("Sauts detectes : %d ; tailles : %s" % (len(sauts), " ".join("%.2f" % di[n - 1] for n in sauts)))
print("Attendus      : %s" % " ".join("%.2f" % (c2 - c1) for c1, c2 in zip([a0] + consignes, consignes)))
th_deg = i - (a - a0)


def passe_bas(x, fc):
    k = 1 - np.exp(-2 * np.pi * fc / fe)
    for _ in range(2):
        y = np.empty_like(x); y[0] = x[0]
        for n in range(1, len(x)):
            y[n] = y[n - 1] + k * (x[n] - y[n - 1])
        x = y[::-1]
    return x


th = np.radians(th_deg); w = np.radians(om); vm = v * C
ok = actif.copy()
for fc in (2.0, 3.0, 5.0):
    thf, wf, vf = passe_bas(th, fc), passe_bas(w, fc), passe_bas(vm, fc)
    alpha = np.gradient(wf, t)
    acc = np.gradient(vf, t)
    c = np.zeros(len(t), bool); c[60:-60] = True; c &= ok
    X = np.c_[thf[c], acc[c], np.ones(c.sum())]
    coef, *_ = np.linalg.lstsq(X, alpha[c], rcond=None)
    r2 = 1 - np.var(alpha[c] - X @ coef) / np.var(alpha[c])
    gL, kacc, b = coef
    print("\nfiltre %.0f Hz : G/L = %.2f /s2, coef acc = %.3f /m (attendu -1/L), R2 = %.2f"
          % (fc, gL, kacc, r2))
    if gL > 0:
        print("   L d'apres G/L = %.3f m ; L d'apres acc = %.3f m" % (G / gL, -1 / kacc if kacc < 0 else float("nan")))
    # correlation entre regresseurs (colinearite)
    print("   correlation th / acc_roues = %.2f" % np.corrcoef(thf[c], acc[c])[0, 1])

print("\nAmplitudes : th %.2f deg (ecart-type), roues %.0f pas/s" % (th_deg.std(), v.std()))
np.savez(r"C:\RB4\modele\excitation_donnees.npz", t=t, th=th, om=w, v=vm, a=a, a0=a0)
