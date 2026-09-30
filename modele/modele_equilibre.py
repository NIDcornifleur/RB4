#!/usr/bin/env python3
# ================================================================
#  RBBB - Modele lineaire de la boucle d'equilibre (MM1 v1.5)
#  Modes propres (frequence, amortissement) selon la hauteur effective L
#  du pendule. Unites internes SI (rad, m, s).
#
#  Etats : th (inclinaison vraie), om (vitesse de chute), v (vitesse roues),
#          vf (vitesse filtree, boucle KV), omf (vitesse de chute filtree, KD),
#          thh (inclinaison estimee par le filtre complementaire)
#  Commande (formules du MM1, converties) :
#     accel_pas = KP*(incl_deg - angleVise_deg) + KD*omf_deg
#     angleVise_deg = -KV * vf_pas
#  Pendule a base acceleree : om' = (G*th - a) / L
#  Filtre complementaire : thh' = om + (th_acc - thh)/TAU_C,
#     th_acc = th + S_ACC * a / G  (acceleration des roues vue par l'accelerometre)
# ================================================================
import numpy as np

G = 9.81
R = 180 / np.pi
D_ROUE = 0.082
C = np.pi * D_ROUE / 1600          # metres par pas
DT = 0.005
TAU_V = DT / 0.05                  # vitesseFiltree = 0.95*... + 0.05*...
TAU_C = DT * 0.996 / 0.004         # filtre complementaire ALPHA = 0.996


def matrice(L, KP=1800, KD=500, KV=0.0015, F=10.0, S_ACC=0.0):
    tau_f = 1 / (2 * np.pi * F) if F > 0 else 1e-4
    # a = A_thh*thh + A_vf*vf + A_omf*omf   (m/s2)
    A_thh = C * KP * R
    A_vf = KP * KV                  # KP * KV * (vf/C) * C
    A_omf = C * KD * R
    a = np.array([0, 0, 0, A_vf, A_omf, A_thh])   # coefficients de a sur l'etat
    M = np.zeros((6, 6))
    M[0, 1] = 1                                    # th' = om
    M[1] = -a / L; M[1, 0] += G / L                # om' = (G th - a)/L
    M[2] = a                                       # v' = a
    M[3, 2] = 1 / TAU_V; M[3, 3] = -1 / TAU_V      # vf'
    M[4, 1] = 1 / tau_f; M[4, 4] = -1 / tau_f      # omf'
    M[5, 1] = 1                                    # thh' = om + (th + S a/G - thh)/TAU_C
    M[5, 0] += 1 / TAU_C
    M[5] += S_ACC * a / G / TAU_C
    M[5, 5] -= 1 / TAU_C
    return M


def modes(M):
    out = []
    for z in np.linalg.eigvals(M):
        if z.imag < -1e-9:
            continue
        w = abs(z)
        f = abs(z.imag) / (2 * np.pi)
        amort = -z.real / w if w > 1e-9 else float("nan")
        out.append((f, amort, z))
    return sorted(out, key=lambda m: m[1])


def afficher(titre, M):
    ms = modes(M)
    stable = all(m[2].real < 1e-9 for m in ms)
    print(titre + ("" if stable else "   *** INSTABLE ***"))
    for f, amort, z in ms:
        if abs(z) < 1e-6:
            continue
        print("    %s  %.2f Hz  amortissement %.2f   (pole %.2f%+.2fj)"
              % ("oscillant" if f > 0.01 else "         ", f, amort, z.real, z.imag))


if __name__ == "__main__":
    for S in (0.0, 1.0, -1.0):
        print("\n======== accelerometre : S_ACC = %+.0f ========" % S)
        for L in (0.15, 0.25, 0.35, 0.5):
            afficher("L = %.2f m" % L, matrice(L, S_ACC=S))
