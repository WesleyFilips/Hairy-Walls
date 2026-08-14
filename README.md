How it works

Two ways to run it: a scriptable CLI (hairy_walls.py), or a desktop GUI (hairy_walls_gui.py) with file pickers, every parameter exposed as a labeled field with tooltips, and settings that persist between runs.

Uses the filament painting feature of your slicer to selectively apply hair using virtual "hairy tools" remapped to one of your real tools. ex: adding a 2nd virtual "hairy tool" in slicer that remaps to use the same head on a single head printer (tool 0) or a 5th virtual tool remapped to tool 0-3 on up to a 4-head toolchanger with the gui (more tools supported in command line)

It parses outer-wall toolpaths directly out of the sliced file (recognizing Cura, PrusaSlicer, SuperSlicer, and OrcaSlicer's type markers), figures out which way is "outward" from the model's own geometry, and inserts small
filament extrusions along the wall at whatever spacing you set — without needing any information about the original 3D model. This can emulate the loops found on some hairy models, or with tuning can generate true straight
hairs.

Feature list

Two placement modes

Wall interrupt — pauses the wall mid-print, grows the hair in place, resumes exactly where it left off.
Hair plugs — prints each wall completely as sliced first, then comes back and applies every hair for that wall in a separate, more efficient pass (staying retracted between consecutive hairs).

Per-hair shape control

Extrude length + amount, plus an optional dry "whip" extension beyond the extruded portion
Root blob (anchors the base) and wall overlap (roots the hair embedded into the wall's thickness rather than just at its surface — hair-plugs mode only, since wall-interrupt is naturally already at full overlap)
Independent feedrates for extruding vs. non-extruding travel
Retraction with automatic matching un-retraction, plus an extra-restart allowance to counter melt-pressure lag
Z-hop to clear the hair before traveling back over it
Length and angle jitter, and randomized starting phase, for a less uniform/mechanical look

Selective application

Restrict by Z-height range
Skip tiny contours (holes, text) below a minimum length
Up to 4 independent "paint-to-fuzz" tool slots — paint regions in your slicer to a tool number, and only those regions get fuzzed. Each slot can also remap to a different real toolhead, so up to 4 different filaments/colors of hair are possible on one model, single-hotend or true toolchanger alike

Correctness under the hood

Curved walls (arc-fitted G2/G3) are handled natively, not skipped
Handles both absolute and relative extrusion modes, hides all injected filament from the rest of the file's E-accounting so nothing downstream breaks
Optional collision avoidance — checks every candidate hair against every other toolpath on the same layer and shrinks or drops it rather than let it cross something
