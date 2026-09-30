# RBBB - essais du filtre KD (MM1 v1.5) : pour chaque frequence, envoi en direct
# (non enregistre), stabilisation, mesure de 20 s, resume.
# Usage : serie_filtre.ps1 [-frequences 0,20,12,8,5]
param([double[]]$frequences = @(0, 20, 12, 8, 5))
$u = "http://192.168.2.226:8000"

function Envoyer($f) {
    Invoke-RestMethod -Method Post -Uri "$u/reglages" -ContentType "application/json" `
        -Body (@{ f = $f } | ConvertTo-Json -Compress) | Out-Null
}

foreach ($f in $frequences) {
    Envoyer $f
    Start-Sleep -Seconds 6
    $e = Invoke-RestMethod "$u/etat"
    if (-not $e.actif) {
        "*** Robot non actif pendant l'essai f=$f : arret, filtre remis a 0"
        Envoyer 0
        exit 1
    }
    "===== f = $f Hz  (MM1 : f=$($e.reglages_mm1.f)) ====="
    ssh RB4 "python3 /home/rj/rbbb_web/analyse_oscillation.py 20 82 && python3 /home/rj/rbbb_web/analyse_bandes.py" |
        Select-String -Pattern 'INCLINAISON|dominante|Hz, ecart|A [0-9.]+ Hz :|ecart-type [0-9.]+ deg/s|energie :|pics au|Attention'
}
Envoyer 0
"Filtre remis a 0 (aucun reglage enregistre)."
