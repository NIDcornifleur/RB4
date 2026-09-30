#!/usr/bin/env python3
# RBBB - repartition de l'oscillation par bandes de frequence
# (a lancer apres analyse_oscillation.py, qui ecrit /tmp/journal_oscillation.csv)
import numpy as np

d = np.loadtxt("/tmp/journal_oscillation.csv", delimiter=",", skiprows=1)
t, i, g, v = d.T
fe = (len(t) - 1) / (t[-1] - t[0])
dt = np.diff(t)
print("Pas de temps du MM1 : min %.3f s, max %.3f s" % (dt.min(), dt.max()))
tu = np.arange(0, t[-1], 1 / fe)
for nom, x, u in (("Vitesse de chute g", g, "deg/s"),
                  ("Vitesse des roues v", v, "pas/s"),
                  ("Inclinaison i", i, "deg")):
    X = np.interp(tu, t, x)
    X = (X - X.mean()) * np.hanning(len(X))
    m = 8 * len(X)
    A = np.abs(np.fft.rfft(X, m)) * 4 / len(X)
    f = np.fft.rfftfreq(m, 1 / fe)
    tot = np.sum(A ** 2)
    print("\n%s : ecart-type %.2f %s" % (nom, x.std(), u))
    print("  energie : " + "   ".join(
        "%g-%g Hz %.0f %%" % (a, b, 100 * np.sum(A[(f >= a) & (f < b)] ** 2) / tot)
        for a, b in ((0, 3), (3, 10), (10, 20), (20, 25.1))))
    M = A * (f > 3)
    s = [j for j in range(1, len(M) - 1) if M[j] >= M[j - 1] and M[j] > M[j + 1]]
    s.sort(key=lambda j: -M[j])
    print("  pics au-dessus de 3 Hz : " + ", ".join("%.2f Hz (%.3f)" % (f[j], M[j]) for j in s[:4]))
