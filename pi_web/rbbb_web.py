#!/usr/bin/env python3
# ================================================================
#  RBBB - Serveur web de commande (Raspberry Pi 3 -> MM1)
#  Version 1.9
#
#  - Video en direct (MJPEG) : camera USB, sinon camera V2 du Pi
#  - Commande de deplacement : vitesse et virage en %
#  - Reglage en direct de KP, KD, KV et de l'angle d'equilibre
#
#  Lien avec le MM1 : /dev/serial0 (GPIO14/15) <-> Serial1 du MM1,
#  115200 bauds, lignes terminees par "\n".
#
#  Pi -> MM1 :
#     m <vitesse> <virage>   vitesse et virage de -100 a 100 (%),
#                            envoye 10 fois par seconde en permanence
#                            (le MM1 s'arrete s'il ne recoit plus rien)
#     p <val> / d <val> / v <val> / a <val> / f <val> / i <val>
#     e <val> / r <val> / b <val>   reglages
#     ?                       demande les reglages actuels
#     c                       refait la calibration du gyroscope (robot couche)
#  MM1 -> Pi :
#     R p=<kp> d=<kd> v=<kv> a=<angle> f=<Hz> i=<ki> e=<ant.> r=<rampe> b=<freinage>
#                                        reponse aux reglages
#     E i=<inclinaison> r=<0|1>          etat (r=1 : equilibre actif)
#       [g=<deg/s> v=<pas/s> t=<ms>]     details envoyes par le MM1 v1.4 (50/s)
#       [n=<deg>]                        terme integral (MM1 v1.7)
#     DEMARRAGE                          le MM1 vient de (re)demarrer
#
#  Page : http://192.168.2.226:8000
#
#  1.1 : bouton d'extinction du Pi, image de la camera tournee de 180 degres
#  1.2 : image de la camera non tournee (camera deplacee) ;
#        journal des 60 dernieres secondes de lignes "E" (page /journal)
#        pour mesurer l'oscillation du robot
#  1.3 : reglage "f" (filtre de la vitesse de chute pour KD, MM1 v1.5),
#        transmis, enregistre et renvoye au MM1 comme p, d, v et a ;
#        "Valeurs d'origine" = reglages valides du 2026-09-28
#  1.4 : bouton "Recalibrer le gyroscope" (commande "c", MM1 v1.6)
#  1.5 : camera USB Innomaker U20CAM-1080p (MJPEG recopie par ffmpeg sans
#        recompression), utilisee en priorite ; camera du Pi si absente
#  1.6 : reglage "i" (terme integral de la boucle de vitesse, MM1 v1.7),
#        transmis, enregistre et renvoye comme les autres ; 0 par defaut
#        (= comportement du MM1 v1.6). Le journal garde aussi "n".
#  1.7 : reglages "e" (anticipation, 0 a 2) et "r" (rampe en pas/s2),
#        MM1 v1.8. Par defaut e 0 et r 1000 (= comportement du MM1 v1.7).
#  1.8 : avertissement "angle d'equilibre a corriger" : robot arrete depuis
#        5 s, KI actif, et le terme integral "n" penche le robot de plus de
#        1 degre en moyenne sur 3 s -> l'angle regle est faux d'environ n.
#        Bouton "Appliquer" (route /corriger_angle) : envoie le nouvel angle
#        et remet le terme integral a zero en meme temps (i 0 puis KI).
#  1.9 : reglage "b" (rampe de freinage en pas/s2, MM1 v1.9) ; 0 par defaut
#        (= meme rampe qu'a l'acceleration, comportement du MM1 v1.8).
# ================================================================

import collections
import io
import json
import os
import subprocess
import threading
import time

import serial
from flask import Flask, Response, jsonify, request, send_from_directory

PORT_SERIE = "/dev/serial0"
VITESSE_SERIE = 115200
PORT_WEB = 8000
DOSSIER = os.path.dirname(os.path.abspath(__file__))
FICHIER_REGLAGES = os.path.join(DOSSIER, "reglages.json")

# Valeurs validees par l'utilisateur le 2026-09-28 (MM1 v1.5, bouton
# "Valeurs d'origine"). Avant : p 1800, d 500, v 0.0015, a 12.4, sans filtre.
REGLAGES_ORIGINE = {"p": 2600.0, "d": 900.0, "v": 0.0007, "a": 15.9, "f": 10.0,
                    "i": 0.0, "e": 0.0, "r": 1000.0,
                    "b": 0.0}

# Limites acceptees depuis la page (protection contre une faute de frappe)
LIMITES = {
    "p": (0.0, 20000.0),
    "d": (0.0, 5000.0),
    "v": (0.0, 0.05),
    "a": (-30.0, 30.0),
    "f": (0.0, 100.0),
    "i": (0.0, 0.01),
    "e": (0.0, 2.0),
    "r": (200.0, 8000.0),
    "b": (0.0, 8000.0),
}

DELAI_ORDRE = 0.5      # s sans ordre de la page -> consigne a zero
PERIODE_ENVOI = 0.1    # s entre deux envois "m" au MM1
TAILLE_JOURNAL = 3000  # lignes "E" gardees (60 s a 50 par seconde)

# Avertissement "angle d'equilibre a corriger" (terme integral du MM1 v1.7)
CONSEIL_ARRET = 5.0        # s sans ordre de deplacement avant de juger
CONSEIL_DUREE = 3.0        # s de mesures moyennees
CONSEIL_SEUIL = 1.0        # degres : au-dela, l'angle est a corriger
CONSEIL_ROUES_MAX = 100.0  # pas/s : roues presque immobiles

# Camera : "auto" = camera USB si branchee, sinon camera du Pi ;
#          "usb" = camera USB seulement ; "pi" = camera du Pi seulement
CAMERA_CHOIX = "auto"
CAMERA_USB = "/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0"
USB_TAILLE = "640x480"  # MJPEG natif : 640x480, 1280x720, 1920x1080 (30 images/s)
USB_GARDER = 2          # une image sur 2 -> 15 images/s vers la page

app = Flask(__name__, static_folder=None)


# ---------------------------------------------------------------- MM1
class LienMM1:
    def __init__(self):
        self.verrou = threading.Lock()
        self.vitesse = 0
        self.virage = 0
        self.dernier_ordre = 0.0
        self.dernier_mouvement = 0.0    # dernier ordre de deplacement non nul
        self.reglages_mm1 = {}          # tels que rapportes par le MM1
        self.inclinaison = None
        self.journal = collections.deque(maxlen=TAILLE_JOURNAL)
        self.actif = False
        self.derniere_reception = 0.0
        self.dernieres_lignes = []
        self.ser = None
        self.reglages_pi = self.charger_reglages()

    # --- reglages enregistres sur le Pi
    def charger_reglages(self):
        try:
            with open(FICHIER_REGLAGES) as f:
                r = json.load(f)
            return {k: float(r.get(k, REGLAGES_ORIGINE[k])) for k in REGLAGES_ORIGINE}
        except (OSError, ValueError):
            return dict(REGLAGES_ORIGINE)

    def enregistrer_reglages(self, r):
        self.reglages_pi = dict(r)
        tmp = FICHIER_REGLAGES + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.reglages_pi, f, indent=2)
        os.replace(tmp, FICHIER_REGLAGES)

    # --- envoi
    def envoyer(self, ligne):
        with self.verrou:
            if self.ser is None:
                return False
            try:
                self.ser.write((ligne + "\n").encode("ascii"))
                return True
            except serial.SerialException:
                return False

    def envoyer_reglages(self, r):
        for k in ("p", "d", "v", "a", "f", "i", "e", "r", "b"):
            if k in r:
                self.envoyer("%s %g" % (k, r[k]))
                time.sleep(0.02)
        self.envoyer("?")

    # --- ordre de deplacement venant de la page
    def ordre(self, vitesse, virage):
        self.vitesse = max(-100, min(100, int(round(vitesse))))
        self.virage = max(-100, min(100, int(round(virage))))
        self.dernier_ordre = time.time()
        if self.vitesse or self.virage:
            self.dernier_mouvement = self.dernier_ordre

    def arret(self):
        self.vitesse = 0
        self.virage = 0

    # --- angle d'equilibre a corriger ?
    def conseil_angle(self):
        # Robot arrete, en equilibre, KI actif : le terme integral "n" penche
        # le robot pour compenser un angle d'equilibre faux. En regime
        # etabli, le vrai point d'equilibre est a peu pres a + n.
        # None s'il n'y a rien a signaler.
        ki = self.reglages_mm1.get("i", 0)
        a = self.reglages_mm1.get("a")
        maintenant = time.time()
        if not self.actif or not ki or a is None:
            return None
        if maintenant - self.dernier_mouvement < CONSEIL_ARRET:
            return None
        lignes = [l for l in list(self.journal) if l[0] > maintenant - CONSEIL_DUREE]
        if len(lignes) < 50:
            return None
        if any(l[6] is None or l[4] is None or not l[5] for l in lignes):
            return None
        n = sum(l[6] for l in lignes) / len(lignes)
        roues = sum(abs(l[4]) for l in lignes) / len(lignes)
        if roues > CONSEIL_ROUES_MAX or abs(n) < CONSEIL_SEUIL:
            return None
        return {"n": round(n, 2), "a": a, "conseil": round(a + n, 1)}

    # --- fils de travail
    def ouvrir(self):
        while self.ser is None:
            try:
                self.ser = serial.Serial(PORT_SERIE, VITESSE_SERIE, timeout=0.2)
            except serial.SerialException as e:
                print("Port serie indisponible :", e, flush=True)
                time.sleep(2)

    def boucle_envoi(self):
        while True:
            if time.time() - self.dernier_ordre > DELAI_ORDRE:
                self.arret()
            self.envoyer("m %d %d" % (self.vitesse, self.virage))
            time.sleep(PERIODE_ENVOI)

    def boucle_reception(self):
        tampon = b""
        while True:
            try:
                tampon += self.ser.read(256)
            except serial.SerialException:
                time.sleep(0.5)
                continue
            while b"\n" in tampon:
                brut, tampon = tampon.split(b"\n", 1)
                # octets nuls parasites au demarrage du Pi : retires
                ligne = brut.replace(b"\x00", b"").decode("ascii", "replace").strip()
                if ligne:
                    self.traiter(ligne)
            if len(tampon) > 1000:
                tampon = b""

    def traiter(self, ligne):
        self.derniere_reception = time.time()
        if ligne.startswith("E "):
            champs = self.champs(ligne)
            try:
                self.inclinaison = float(champs.get("i"))
            except (TypeError, ValueError):
                pass
            self.actif = champs.get("r") == "1"
            self.noter_journal(champs)
            return
        if not ligne.startswith("R "):   # les reglages sont deja affiches a part
            self.dernieres_lignes = (self.dernieres_lignes + [ligne])[-8:]
        if ligne.startswith("R "):
            for k, v in self.champs(ligne).items():
                try:
                    self.reglages_mm1[k] = float(v)
                except ValueError:
                    pass
        elif ligne.startswith("DEMARRAGE"):
            # Le MM1 a redemarre avec les valeurs de son programme :
            # on lui renvoie les reglages enregistres sur le Pi.
            threading.Thread(target=self.renvoyer_apres_demarrage, daemon=True).start()

    def noter_journal(self, champs):
        # [heure du Pi, heure du MM1 en ms, inclinaison, vitesse de chute,
        #  vitesse des roues, equilibre actif, terme integral] ; None si le
        # MM1 n'envoie pas la valeur (< v1.4 ; terme integral : < v1.7)
        def nombre(k):
            try:
                return float(champs[k])
            except (KeyError, ValueError):
                return None
        self.journal.append([round(time.time(), 3), nombre("t"), nombre("i"),
                             nombre("g"), nombre("v"), champs.get("r") == "1",
                             nombre("n")])

    def renvoyer_apres_demarrage(self):
        time.sleep(0.5)
        self.envoyer_reglages(self.reglages_pi)

    @staticmethod
    def champs(ligne):
        d = {}
        for morceau in ligne.split()[1:]:
            if "=" in morceau:
                k, v = morceau.split("=", 1)
                d[k] = v
        return d

    def demarrer(self):
        self.ouvrir()
        threading.Thread(target=self.boucle_reception, daemon=True).start()
        threading.Thread(target=self.boucle_envoi, daemon=True).start()
        # Au demarrage du serveur : le MM1 recoit les reglages du Pi
        threading.Thread(target=self.renvoyer_apres_demarrage, daemon=True).start()


mm1 = LienMM1()


# ---------------------------------------------------------------- camera
class SortieVideo(io.BufferedIOBase):
    def __init__(self):
        self.image = None
        self.condition = threading.Condition()

    def write(self, buf):
        with self.condition:
            self.image = buf
            self.condition.notify_all()
        return len(buf)


video = SortieVideo()
camera_ok = False
camera_source = None      # "usb" ou "pi"


def usb_presente():
    return os.path.exists(CAMERA_USB)


def boucle_camera_usb():
    # ffmpeg recopie le MJPEG de la camera sans le recompresser (-c copy) ;
    # on decoupe les images (FFD8 ... FFD9) et on en garde une sur USB_GARDER.
    # Si la camera se debranche ou si ffmpeg s'arrete, on recommence.
    commande = ["ffmpeg", "-hide_banner", "-loglevel", "error",
                "-f", "v4l2", "-input_format", "mjpeg",
                "-video_size", USB_TAILLE, "-framerate", "30",
                "-i", CAMERA_USB, "-c", "copy", "-f", "mjpeg", "pipe:1"]
    while True:
        if not usb_presente():
            time.sleep(2)
            continue
        proc = subprocess.Popen(commande, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        tampon = b""
        compteur = 0
        try:
            while True:
                morceau = proc.stdout.read(65536)
                if not morceau:
                    break
                tampon += morceau
                while True:
                    d = tampon.find(b"\xff\xd8")
                    if d < 0:
                        tampon = b""
                        break
                    f = tampon.find(b"\xff\xd9", d + 2)
                    if f < 0:
                        tampon = tampon[d:]
                        break
                    image = tampon[d:f + 2]
                    tampon = tampon[f + 2:]
                    compteur += 1
                    if compteur % USB_GARDER == 0:
                        video.write(image)
                if len(tampon) > 2_000_000:     # securite : jamais de fin d'image
                    tampon = b""
        finally:
            proc.kill()
            proc.wait()
        print("Camera USB : flux interrompu, nouvel essai dans 2 s", flush=True)
        time.sleep(2)


def demarrer_camera():
    global camera_ok, camera_source
    if CAMERA_CHOIX in ("auto", "usb") and usb_presente():
        threading.Thread(target=boucle_camera_usb, daemon=True).start()
        camera_ok = True
        camera_source = "usb"
        print("Camera USB utilisee :", CAMERA_USB, USB_TAILLE, flush=True)
        return
    if CAMERA_CHOIX == "usb":
        print("Camera USB absente", flush=True)
        return
    try:
        from picamera2 import Picamera2
        from picamera2.outputs import FileOutput
        try:
            from picamera2.encoders import MJPEGEncoder as Encodeur
        except ImportError:
            from picamera2.encoders import JpegEncoder as Encodeur
        from libcamera import Transform
        cam = Picamera2()
        cam.configure(cam.create_video_configuration(
            main={"size": (640, 480)},
            # camera deplacee : image a l'endroit, sans rotation
            # (avant : transform=Transform(hflip=1, vflip=1) pour 180 degres)
            controls={"FrameDurationLimits": (66666, 66666)}))  # 15 images/s
        cam.start_recording(Encodeur(), FileOutput(video))
        camera_ok = True
        camera_source = "pi"
    except Exception as e:  # la commande doit marcher meme sans camera
        print("Camera indisponible :", e, flush=True)


def flux_video():
    while True:
        with video.condition:
            video.condition.wait(timeout=2)
            image = video.image
        if image is None:
            continue
        yield (b"--image\r\nContent-Type: image/jpeg\r\nContent-Length: "
               + str(len(image)).encode() + b"\r\n\r\n" + image + b"\r\n")


# ---------------------------------------------------------------- pages
@app.route("/")
def accueil():
    return send_from_directory(DOSSIER, "index.html")


@app.route("/video")
def page_video():
    if not camera_ok:
        return "Camera indisponible", 503
    return Response(flux_video(), mimetype="multipart/x-mixed-replace; boundary=image")


@app.route("/ordre", methods=["POST"])
def page_ordre():
    d = request.get_json(force=True, silent=True) or {}
    try:
        mm1.ordre(float(d.get("vitesse", 0)), float(d.get("virage", 0)))
    except (TypeError, ValueError):
        mm1.arret()
    return jsonify(ok=True)


@app.route("/arret", methods=["POST"])
def page_arret():
    mm1.arret()
    mm1.dernier_ordre = 0.0
    mm1.envoyer("m 0 0")
    return jsonify(ok=True)


def lire_valeurs(d):
    r = {}
    for k, (mini, maxi) in LIMITES.items():
        if k in d and d[k] not in (None, ""):
            v = float(d[k])
            if not (mini <= v <= maxi):
                raise ValueError("%s hors limites (%g a %g)" % (k.upper(), mini, maxi))
            r[k] = v
    return r


@app.route("/reglages", methods=["POST"])
def page_reglages():
    d = request.get_json(force=True, silent=True) or {}
    try:
        r = lire_valeurs(d)
    except (TypeError, ValueError) as e:
        return jsonify(ok=False, erreur=str(e)), 400
    mm1.envoyer_reglages(r)
    return jsonify(ok=True)


@app.route("/enregistrer", methods=["POST"])
def page_enregistrer():
    d = request.get_json(force=True, silent=True) or {}
    try:
        r = lire_valeurs(d)
    except (TypeError, ValueError) as e:
        return jsonify(ok=False, erreur=str(e)), 400
    complet = dict(mm1.reglages_pi)
    complet.update(r)
    mm1.enregistrer_reglages(complet)
    mm1.envoyer_reglages(complet)
    return jsonify(ok=True, reglages=complet)


@app.route("/origine", methods=["POST"])
def page_origine():
    mm1.envoyer_reglages(REGLAGES_ORIGINE)
    return jsonify(ok=True, reglages=REGLAGES_ORIGINE)


@app.route("/corriger_angle", methods=["POST"])
def page_corriger_angle():
    c = mm1.conseil_angle()
    if c is None:
        return jsonify(ok=False, erreur="Plus d'écart à corriger (ou robot en mouvement)"), 409
    lo, hi = LIMITES["a"]
    nouveau = max(lo, min(hi, c["conseil"]))
    ki = mm1.reglages_mm1.get("i", 0)
    # Nouvel angle et remise a zero du terme integral dans le meme tour de
    # boucle du MM1 (sinon la correction compterait deux fois) ; KI est
    # remis 50 ms plus tard (le MM1 fait un tour toutes les 5 ms).
    mm1.envoyer("a %g" % nouveau)
    mm1.envoyer("i 0")
    time.sleep(0.05)
    mm1.envoyer("i %g" % ki)
    time.sleep(0.02)
    mm1.envoyer("?")
    return jsonify(ok=True, angle=nouveau)


@app.route("/lire", methods=["POST"])
def page_lire():
    mm1.envoyer("?")
    return jsonify(ok=True)


@app.route("/calibrer", methods=["POST"])
def page_calibrer():
    # Le MM1 refuse si l'equilibre est actif ; ses messages ("I ...")
    # s'affichent sur la page.
    if mm1.actif:
        return jsonify(ok=False, erreur="Couchez d'abord le robot (équilibre actif)"), 409
    mm1.envoyer("c")
    return jsonify(ok=True)


@app.route("/eteindre", methods=["POST"])
def page_eteindre():
    # Arret du deplacement, puis extinction propre du Pi.
    # Le MM1 continue seul : sans ordre, il reste en equilibre sur place.
    mm1.arret()
    mm1.dernier_ordre = 0.0
    mm1.envoyer("m 0 0")
    mm1.envoyer("?")      # ligne neutre : laisse le temps au "m 0 0" de partir
    subprocess.Popen(["sudo", "-n", "/sbin/poweroff"])
    return jsonify(ok=True)


@app.route("/etat")
def page_etat():
    return jsonify(
        mm1_repond=(time.time() - mm1.derniere_reception) < 2.0,
        inclinaison=mm1.inclinaison,
        actif=mm1.actif,
        reglages_mm1=mm1.reglages_mm1,
        reglages_pi=mm1.reglages_pi,
        vitesse=mm1.vitesse,
        virage=mm1.virage,
        camera=camera_ok,
        camera_source=camera_source,
        messages=mm1.dernieres_lignes,
        conseil_angle=mm1.conseil_angle(),
    )


@app.route("/journal")
def page_journal():
    # Lignes "E" recues depuis ?depuis=<heure du Pi> (60 dernieres secondes au plus)
    try:
        depuis = float(request.args.get("depuis", 0))
    except ValueError:
        depuis = 0.0
    lignes = [l for l in list(mm1.journal) if l[0] > depuis]
    return jsonify(colonnes=["t_pi", "t_mm1", "i", "g", "v", "r", "n"], lignes=lignes)


if __name__ == "__main__":
    mm1.demarrer()
    demarrer_camera()
    app.run(host="0.0.0.0", port=PORT_WEB, threaded=True)
