# 5-inch wheel for a 28BYJ48 stepper

Print two copies of `wheel-127mm.stl`. All STL coordinates are millimetres.
This is a proposed design, not a physically tested part. The exact -007 motor
shaft and cardboard body have not been measured.

## Dimensions

- Plastic outside diameter: 127 mm (5 inches), before rubber bands.
- Rim width: 10 mm; total axial width including hub: 14 mm.
- Six spokes, 6 mm wide and 4 mm thick; central hub diameter: 20 mm.
- Two rubber-band grooves: 0.8 mm deep, 2.4 mm wide at the bottom,
  with sloping sides. Use bands narrow enough to sit in the grooves.
- Blind double-flat socket: 6 mm deep, nominal 5 mm round diameter,
  3 mm between flats. Default total clearance is +0.15 mm in each dimension
  (5.15 mm diameter and 3.15 mm across flats). Entry has a 0.4 mm chamfer.
- Solid CAD volume approximately 32 cm³; slicer infill affects actual mass.

## Fit first

Print the three small fit-test hubs before either wheel. File names indicate
the total clearance added to both shaft dimensions: 0.00, 0.15, and 0.30 mm.
Keep them identifiable on the print plate; they have no embossed labels.
Use the same material and slicer settings as the wheels.

The motor-facing side is the raised hub with the socket opening. Try a gentle
hand fit. Do not hammer or force the shaft into a tight hole: support the motor
and avoid loading its gearbox. Choose a snug fit that neither rocks nor slips.
If all three are loose or tight, measure the shaft before changing the design.
No set screw or mechanical axial retainer is included; this design relies on
friction. Check that it stays seated during reverse and turning tests.

The common shaft dimensions are a starting assumption, not verification of
your particular motor:
https://www.gentiam.com/wp-content/uploads/2022/09/GentiamElectronics-28BYJ-48-Stepper-Motor-Datasheet-Rev20220925-final.pdf
https://keszoox.com/products/stepper-motor-with-uln2003a-driver-board

## Printing

- Put the broad flat face on the build plate, with the socket facing upward.
- Starting settings: PLA, 0.20 mm layers, 4 walls, 5 top/bottom layers,
  20–30% infill. Inspect the sliced preview before printing.
- Supports should not be needed in this orientation. Groove slopes are 45°.
- Print two identical wheels; no mirroring is needed.
- Stretch rubber bands into the grooves. Rubber must protrude beyond the
  plastic rim to provide traction: band thickness must exceed the 0.8 mm recess.
  Do not put bands on so tightly that they distort the wheel.

## Cardboard body and motor mounting

Measure the diameter with bands fitted. Leave at least 2–3 mm clearance around
the rotating rubber and wheel sides. A cutout or wheel well must accommodate
that full diameter plus clearance, not merely the 127 mm plastic diameter.
The bottom of the box must sit above the floor; use a caster to support the
third point and keep the phone/Pi low. Attach motors to a stiff plate or bracket
rather than relying on flexible cardboard to hold their alignment.

Test one wheel, then both with the actual payload on a smooth surface before
finishing the body. These large wheels require about 3.2 times the motor torque
of 40 mm wheels for the same tangential force. Payload capacity is not verified.

## Change the size or fit

From the repository root, using Python 3 (no extra packages):

```sh
python3 hardware/stepper-wheel/generate.py --diameter 127 --clearance 0.15
```

For example, choose `--clearance 0.30` if that test hub fits best. The generator
overwrites the wheel STL with that diameter and regenerates all three coupons.
Shaft dimensions, spoke and groove geometry are defined in `generate.py`.
Do not scale the whole STL to change wheel diameter: that also changes the bore.
The generator checks nonzero triangles, edge closure, consistent winding and
positive mesh volume. Those checks do not establish physical fit or strength.
