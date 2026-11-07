# Roller — 350 mm, printable

A 350 mm roller for the intake, sized for a REV 5 mm hex shaft, made from two printed halves
and a collar. 350 mm does not fit a typical 220–256 mm printer bed, so the drum is split in two.

| File | Print | Size |
|---|---|---|
| `roller_segment.stl` | **2 off** | 50 mm dia × 175 mm |
| `roller_collar.stl` | 1 off | 54 mm dia × 34 mm |
| `roller_drawing.png` | — | dimensioned view |

Bore is a 5.2 mm across-flats hexagon, which gives a light sliding fit on the REV shaft and
carries torque without a grub screw.

## Printing

- **Orientation:** upright, bore vertical. The hex prints perfectly with no supports, and the
  layers run around the drum rather than across it.
- **Walls:** 4 perimeters. **Infill:** 15–25 %. **Layer:** 0.2 mm.
- **Material:** PETG for grip and toughness; PLA is fine for a test fit.
- **Mass:** about 130 g per segment at those settings, 12 g for the collar.
- Both meshes were checked for watertightness before export — every edge is shared by exactly
  two triangles.

## Assembly

1. Slide both segments onto one continuous 350 mm hex shaft.
2. Press the collar over the joint so the halves cannot rock against each other.
3. A drop of superglue at the joint is enough if the collar is a tight fit; leave the ends free.

## Regenerating

```
python3 make_roller.py
```

Diameters, length, bore and collar size are constants at the top of that file. Change
`SEG_LEN` to 116.7 to make three shorter pieces, or `BORE_AF` to 5.4 for a looser fit.
