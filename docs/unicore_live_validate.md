# Unicore GNSS Live Validate

Le validateur live UM98x/Unicore n'est plus maintenu dans MowgliNext.

La version de reference est maintenant dans le repo `UM982Driver`:
- [docs/live_validate.md](/home/pepeuch/Documents/vscode/tondeuse/UM982Driver/docs/live_validate.md)
- [tools/um982_live_validate.py](/home/pepeuch/Documents/vscode/tondeuse/UM982Driver/tools/um982_live_validate.py)

## Depuis MowgliNext

Utiliser le wrapper robot:

```bash
./sensors/unicore/validate_live.sh --help
```

Exemple runtime leger:

```bash
./sensors/unicore/validate_live.sh \
  --port /dev/gps \
  --baud 921600 \
  --profile normal \
  --format ascii \
  --duration 30 \
  --apply-profile-logs
```

Le validateur du driver gere maintenant une table de syntaxe N4 par
message, avec fallback automatique seulement si le firmware rejette la
forme canonique:
- `LOG ... ONTIME` pour `GPGGA` et `PVTSLN`
- periode directe pour `BESTNAV`, `RTKSTATUS`, `GPHPR`
- `ONCHANGED` pour `RTCMSTATUS` et `GPHPR2`

Exemple survey hybride:

```bash
./sensors/unicore/validate_live.sh \
  --port /dev/gps \
  --baud 921600 \
  --profile survey \
  --format hybrid \
  --enable-raw \
  --apply-profile-config \
  --apply-profile-logs \
  --duration 60
```

## Note

MowgliNext garde seulement l'integration robot et les chemins locaux. Toute la logique de validation live, de reset, de profils et de parsing de capture appartient desormais au repo `UM982Driver`.
