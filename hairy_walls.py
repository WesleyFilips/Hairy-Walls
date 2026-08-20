#!/usr/bin/env python3
"""
hairy_walls.py
==============

Hairy Walls -- a parametric G-code post-processor that grows small
outward loops of filament ("fuzz" / "hair") along outer-wall perimeters,
in the spirit of the "hairy lion" printable but usable on ANY sliced
model.

How it works
------------
While streaming through the file, the script watches for slicer "TYPE"
comments (";TYPE:WALL-OUTER", ";TYPE:External perimeter", etc. -- Cura,
PrusaSlicer, SuperSlicer and OrcaSlicer all emit one of these) to find
outer-wall extrusion moves, including curved sections printed as G2/G3
arcs (which are expanded into short line segments internally so a wall
loop stays one continuous contour through curves). It buffers each
contour until it ends, then:

  1. Computes the contour's winding direction with the shoelace formula,
     which tells us which perpendicular direction points away from the
     model at each edge -- no knowledge of the source mesh required.
  2. Sanity-checks that direction against the contour's centroid (flips
     it if a loop would point back toward the model's own interior) as
     a backstop for messy/irregular contours.
  3. Walks the contour accumulating arc length. Every `--spacing` mm it
     splits the current edge at that point and inserts a little
     "out-and-back" detour: travel `--length` mm outward while extruding
     `--extrude` mm of filament, then return, then resumes the original
     wall path exactly where it left off.

To keep the rest of the file byte-identical (so slicer-computed
retractions, cooling, etc. still line up), the extra filament injected by
each loop is "hidden" from the printer's absolute E accounting with a
`G92 E<original-target>` reset immediately after each loop (only needed
in absolute-extrusion / M82 files; M83 relative-extrusion files need no
such trick since deltas are self-contained).

Selective fuzz via multi-material painting
-------------------------------------------
If you want fuzz on only *some* faces of the model, paint those regions
to a second extruder/filament in PrusaSlicer's or OrcaSlicer's paint-on
tool (it can map to the same physical filament -- you're just using it
as a marker). The slicer will emit T0/T1 tool-change commands at the
painted boundaries. Pass --fuzzy-tool-1 4 (or whichever tool number you
painted) and the script will only add loops while that tool is active,
leaving everything else smooth. Up to 4 independent fuzzy-tool/remap
pairs are supported, for up to 4 different fuzzy colors.

Usage
-----
    python3 hairy_walls.py input.gcode output.gcode \\
        --spacing 4 --length 1.6 --extrude 0.35

Run with -h for the full parameter list.
"""

import argparse
import math
import random
import re
import sys

try:
    import gcode3mf
except ImportError:
    gcode3mf = None

__version__ = "2.1.1"
# Changelog:
#   2.1.1 - --return-extrude now interacts with --back-travel: with
#           back-travel 0 (default) it still extrudes over the whole
#           return-to-wall trip as before; with back-travel above 0, it
#           extrudes only during that backward leg (right at the tip)
#           instead, and the remaining trip back to the wall stays a
#           dry, retracted move.
#   2.1.0 - .gcode.3mf (Bambu/OrcaSlicer sliced-plate) support via the
#           new gcode3mf.py module, wired into both the CLI (--plate for
#           multi-plate archives) and the GUI. Also broadened TYPE/LAYER
#           comment detection to include ;FEATURE: and ;CHANGE_LAYER
#           alongside the existing ;TYPE:/;LAYER_CHANGE conventions.
#   2.0.0 - new defaults that change behavior for existing command lines:
#           min-contour-length 8->1, length 1.5->2, dry-length 0->4,
#           z-hop 0->1, retract 0->0.4, extra-restart 0->0.02,
#           dry-feedrate now a concrete 2400 instead of falling back to
#           --feedrate, collision-margin 0->1, and random-phase /
#           avoid-collisions / fan-boost now default ON (each has a new
#           --no-<flag> to turn it back off).
#   1.0.0 - first version-tagged release. Wall-interrupt and hair-plugs
#           modes, arc support, collision avoidance, up to 4 independent
#           fuzzy-tool/remap slots, root/retract/extra-restart/z-hop,
#           dry-length, back-travel (hooks), wall-overlap (hair-plugs
#           only), fan boost.

MOVE_RE = re.compile(r'^(G0|G1)\b', re.IGNORECASE)
ARC_RE = re.compile(r'^(G2|G3)\b', re.IGNORECASE)
COORD_RE = re.compile(r'([XYZEF])(-?[0-9.]+)', re.IGNORECASE)
ARC_COORD_RE = re.compile(r'([XYZIJEF])(-?[0-9.]+)', re.IGNORECASE)
TYPE_RE = re.compile(r';\s*(?:TYPE|FEATURE)\s*:\s*(.+)', re.IGNORECASE)
LAYER_RE = re.compile(r';\s*(?:LAYER|CHANGE_LAYER|LAYER_CHANGE)\b', re.IGNORECASE)
TOOL_RE = re.compile(r'^T(\d+)\b', re.IGNORECASE)
FAN_ON_RE = re.compile(r'^M106\b', re.IGNORECASE)
FAN_OFF_RE = re.compile(r'^M107\b', re.IGNORECASE)
FAN_S_RE = re.compile(r'\bS(-?[0-9.]+)', re.IGNORECASE)
# same thing but anchored on the raw (unstripped) line, so a remap can
# rewrite the tool number in place while preserving indentation/comments
TOOL_LINE_RE = re.compile(r'^(\s*T)(\d+)\b', re.IGNORECASE)

# Slicer comment fragments that mean "this is the outer/external wall".
OUTER_WALL_KEYWORDS = ('outer', 'external')

# Arc segmentation target: roughly one segment per this many mm of arc length.
ARC_SEGMENT_LENGTH = 0.4
ARC_MIN_SEGMENTS = 2
ARC_MAX_SEGMENTS = 240


# --------------------------------------------------------------------------
# small helpers
# --------------------------------------------------------------------------

def fmt(v):
    """Trim floats to something gcode-clean (5 decimals, no trailing junk)."""
    s = f"{v:.5f}".rstrip('0').rstrip('.')
    return s if s not in ('', '-0') else '0'


def dist2d(a, b):
    return math.hypot(b['x'] - a['x'], b['y'] - a['y'])


def lerp_point(a, b, t):
    return {
        'x': a['x'] + (b['x'] - a['x']) * t,
        'y': a['y'] + (b['y'] - a['y']) * t,
        'z': a['z'] + (b['z'] - a['z']) * t,
        'e': a['e'] + (b['e'] - a['e']) * t,   # base (fuzz-free) extrusion target
        'f': b['f'],
    }


def signed_area2(points):
    """2x signed polygon area (shoelace). Positive => CCW in standard XY."""
    a = 0.0
    for i in range(len(points) - 1):
        x1, y1 = points[i]['x'], points[i]['y']
        x2, y2 = points[i + 1]['x'], points[i + 1]['y']
        a += x1 * y2 - x2 * y1
    return a


def arc_points(start, end_x, end_y, i_off, j_off, clockwise):
    """Expand a G2/G3 arc into a list of (x, y, t) samples, t in (0, 1]
    giving the fractional progress along the arc for E interpolation."""
    cx = start['x'] + i_off
    cy = start['y'] + j_off
    r = math.hypot(start['x'] - cx, start['y'] - cy)
    if r < 1e-9:
        return [(end_x, end_y, 1.0)]

    start_angle = math.atan2(start['y'] - cy, start['x'] - cx)
    end_angle = math.atan2(end_y - cy, end_x - cx)

    if clockwise:
        while end_angle >= start_angle:
            end_angle -= 2 * math.pi
    else:
        while end_angle <= start_angle:
            end_angle += 2 * math.pi

    sweep = end_angle - start_angle
    arc_len = abs(sweep) * r
    segments = int(arc_len / ARC_SEGMENT_LENGTH)
    segments = max(ARC_MIN_SEGMENTS, min(ARC_MAX_SEGMENTS, segments))

    pts = []
    for k in range(1, segments + 1):
        t = k / segments
        ang = start_angle + sweep * t
        pts.append((cx + r * math.cos(ang), cy + r * math.sin(ang), t))
    # make sure the endpoint is exact, not a trig-rounded approximation
    pts[-1] = (end_x, end_y, 1.0)
    return pts


# --------------------------------------------------------------------------
# core contour -> fuzzed gcode
# --------------------------------------------------------------------------

class Emitter:
    """Writes gcode lines, translating our internal absolute x/y/z/e state
    back into whatever coordinate mode (absolute/relative) the source file
    was using at this point."""

    def __init__(self, start_state, mode_xyz_abs, mode_e_abs):
        self.last = dict(start_state)
        self.mode_xyz_abs = mode_xyz_abs
        self.mode_e_abs = mode_e_abs
        self.lines = []

    def move(self, gcmd, x, y, z, e, f, comment=None):
        parts = [gcmd]
        if self.mode_xyz_abs:
            if x != self.last['x']:
                parts.append(f"X{fmt(x)}")
            if y != self.last['y']:
                parts.append(f"Y{fmt(y)}")
            if z != self.last['z']:
                parts.append(f"Z{fmt(z)}")
        else:
            if x != self.last['x']:
                parts.append(f"X{fmt(x - self.last['x'])}")
            if y != self.last['y']:
                parts.append(f"Y{fmt(y - self.last['y'])}")
            if z != self.last['z']:
                parts.append(f"Z{fmt(z - self.last['z'])}")

        if self.mode_e_abs:
            parts.append(f"E{fmt(e)}")
        else:
            parts.append(f"E{fmt(e - self.last['e'])}")

        if f != self.last['f']:
            parts.append(f"F{fmt(f)}")

        if comment:
            parts.append(f"; {comment}")

        self.lines.append(' '.join(parts) + '\n')
        self.last = {'x': x, 'y': y, 'z': z, 'e': e, 'f': f}

    def reset_e_register(self, base_e):
        """Emit G92 E<base_e> so downstream *original* absolute-E lines
        (which we pass through untouched) stay valid despite the extra
        filament we just laid down. Only meaningful in absolute-E mode."""
        if self.mode_e_abs:
            self.lines.append(f"G92 E{fmt(base_e)} ; hairy_walls: hide injected filament\n")
        self.last['e'] = base_e


def fuzzify_contour(contour, params, mode_xyz_abs, mode_e_abs, rng,
                     grid=None, own_contour_id=None, current_fan_speed=0.0):
    """Dispatches to the selected loop-placement mode. See
    _fuzzify_wall_interrupt (loops spliced into the wall path itself, as
    it prints) and _fuzzify_hair_plugs (wall prints normally first, then
    every loop for this contour is applied as a separate pass)."""
    if params.mode == 'hair-plugs':
        return _fuzzify_hair_plugs(contour, params, mode_xyz_abs, mode_e_abs, rng, grid, own_contour_id,
                                    current_fan_speed)
    return _fuzzify_wall_interrupt(contour, params, mode_xyz_abs, mode_e_abs, rng, grid, own_contour_id,
                                    current_fan_speed)


def fan_restore_line(speed):
    """M106/M107 line that puts the fan back exactly where it was."""
    if speed <= 0:
        return "M107 ; hairy_walls: restore fan\n"
    return f"M106 S{fmt(speed)} ; hairy_walls: restore fan\n"


def _fuzzify_wall_interrupt(contour, params, mode_xyz_abs, mode_e_abs, rng,
                             grid=None, own_contour_id=None, current_fan_speed=0.0):
    """contour: list of entries in original file order. Most are point dicts
    with absolute x,y,z,e,f,tool (e = original, fuzz-free extrusion target).
    Some may be {'marker': True, 'raw_line': ...} for a T-command that fell
    inside this wall path -- these carry no geometry and must be spliced
    back into the output at the same relative position, untouched.

    If `grid` is given (a SegGrid covering the whole layer's toolpaths),
    each candidate loop is checked against every OTHER toolpath in the
    layer before being committed; a colliding loop is shrunk in steps and,
    failing that, dropped entirely rather than crossing another path."""

    geom_points = [p for p in contour if not p.get('marker')]

    if len(geom_points) < 2:
        return [p['raw_line'] for p in contour if p.get('raw_line')]

    fuzzy_map = get_fuzzy_map(params)

    total_len = sum(dist2d(geom_points[i], geom_points[i + 1]) for i in range(len(geom_points) - 1))
    if total_len < params.min_contour_length:
        return [p['raw_line'] for p in contour if p.get('raw_line')]

    # winding/centroid computed over the FULL closed wall path (all tools
    # together) -- this must never be based on a partial fragment, or the
    # outward direction becomes unreliable right at a tool-change boundary.
    ccw = signed_area2(geom_points) > 0
    cx = sum(p['x'] for p in geom_points) / len(geom_points)
    cy = sum(p['y'] for p in geom_points) / len(geom_points)

    em = Emitter(geom_points[0], mode_xyz_abs, mode_e_abs)

    spacing = max(params.spacing, 0.1)
    dist_since_last = rng.uniform(0, spacing) if params.random_phase else spacing / 2.0
    loops_added = 0
    loops_skipped = 0

    prev = None
    for entry in contour:
        if entry.get('marker'):
            # splice the T-command back in at its correct position
            em.lines.append(entry['raw_line'])
            continue

        if prev is None:
            prev = entry
            continue

        a, b = prev, entry
        seg_len = dist2d(a, b)
        if seg_len <= 1e-9:
            prev = b
            continue

        # gate LOOP INSERTION by tool, but the contour itself was never
        # broken for tool changes, so geometry stays correct either way.
        edge_tool = b.get('tool')
        edge_ok = (not fuzzy_map) or (edge_tool in fuzzy_map)

        if not edge_ok:
            # no fuzz on this edge; just let elapsed distance keep accruing
            # (capped so a long non-fuzzy stretch doesn't cause a pile-up
            # of loops right as it re-enters a fuzzy edge)
            dist_since_last = min(dist_since_last + seg_len, spacing)
            em.move('G1', b['x'], b['y'], b['z'], b['e'], b['f'])
            prev = b
            continue

        tx, ty = (b['x'] - a['x']) / seg_len, (b['y'] - a['y']) / seg_len
        if ccw:
            nx, ny = ty, -tx     # outward normal for a CCW-wound contour
        else:
            nx, ny = -ty, tx     # outward normal for a CW-wound contour

        walked = 0.0
        while True:
            remaining = seg_len - walked
            need = spacing - dist_since_last
            if need > remaining:
                dist_since_last += remaining
                break

            walked += need
            t = walked / seg_len
            split = lerp_point(a, b, t)

            # sanity check: the loop should point away from the contour's
            # own centroid. if it doesn't, the local edge normal is
            # untrustworthy here (e.g. broken/open contour) -- flip it.
            snx, sny = nx, ny
            if (snx * (split['x'] - cx) + sny * (split['y'] - cy)) < 0:
                snx, sny = -snx, -sny

            # jitter scales the TOTAL outward reach (extrude leg + dry leg
            # together) so their ratio -- and so the look of the hair --
            # stays consistent even as length varies loop to loop. When
            # back-travel is active, dry-length is disabled entirely (the
            # tip goes backward toward the part instead of further out),
            # so the whole reach is just the extrude leg.
            back_travel_active = params.back_travel > 0
            base_total = params.length if back_travel_active else (params.length + params.dry_length)
            jittered_total = base_total * (1.0 + rng.uniform(-params.length_jitter, params.length_jitter))
            jittered_total = max(jittered_total, 0.0)
            if back_travel_active:
                extrude_ratio = 1.0
            else:
                extrude_ratio = (params.length / base_total) if base_total > 1e-9 else 1.0

            angle = math.radians(rng.uniform(-params.angle_jitter, params.angle_jitter))
            onx = snx * math.cos(angle) - sny * math.sin(angle)
            ony = snx * math.sin(angle) + sny * math.cos(angle)

            chosen_total = None
            if grid is None:
                chosen_total = jittered_total
            else:
                for frac in SHRINK_STEPS:
                    cand_total = jittered_total * frac
                    tip = (split['x'] + onx * cand_total, split['y'] + ony * cand_total)
                    if not loop_collides(split, tip, grid, own_contour_id, params.collision_margin):
                        chosen_total = cand_total
                        break

            if chosen_total is None:
                # every shrink step still collides -- drop this loop, but
                # still visit the split point so the wall path is unbroken
                em.move('G1', split['x'], split['y'], split['z'], split['e'], split['f'])
                loops_skipped += 1
                dist_since_last = 0.0
                continue

            # move up to the split point along the (unmodified) wall path
            em.move('G1', split['x'], split['y'], split['z'], split['e'], split['f'])

            if params.fan_boost:
                em.lines.append("M106 S255 ; hairy_walls: fan boost\n")

            extrude_len = chosen_total * extrude_ratio
            dry_len = chosen_total - extrude_len
            dry_feedrate = params.dry_feedrate or params.feedrate

            # NB: no wall-overlap dip here. In this mode the wall path is
            # already paused exactly where it stopped -- that IS the
            # equivalent of a full-overlap root, with no separate travel
            # needed to reach it. --wall-overlap only matters in
            # hair-plugs mode, where hairs are approached from outside
            # via a travel move and need somewhere to dig back in.

            # root: a small blob laid down at the wall itself, no travel,
            # before heading outward -- anchors the base of the hair
            root_e = split['e']
            if params.root_extrude > 0:
                root_e = split['e'] + params.root_extrude
                em.move('G1', split['x'], split['y'], split['z'], root_e, params.feedrate,
                        comment='fuzz root')

            # leg 1: outward, extruding
            ext_x = split['x'] + onx * extrude_len
            ext_y = split['y'] + ony * extrude_len
            out_e = root_e + params.extrude
            em.move('G1', ext_x, ext_y, split['z'], out_e, params.feedrate, comment='fuzz out (extrude)')

            last_x, last_y = ext_x, ext_y
            travel_e = out_e
            return_extrude_used = False

            # back-travel: drag the tip backward toward the part, before
            # retracting -- curls the still-soft tip into a hook shape
            # (useful for hook-and-loop fastener hairs). Disables the dry
            # leg for this loop when active. --return-extrude, if set, is
            # extruded DURING this backward leg (over just the back-travel
            # distance) instead of over the whole return-to-wall trip.
            if back_travel_active:
                last_x = ext_x - onx * params.back_travel
                last_y = ext_y - ony * params.back_travel
                if params.return_extrude > 0:
                    travel_e = out_e + params.return_extrude
                    em.move('G1', last_x, last_y, split['z'], travel_e, params.feedrate,
                            comment='fuzz back-travel (hook, extruding)')
                    return_extrude_used = True
                else:
                    em.move('G1', last_x, last_y, split['z'], travel_e, dry_feedrate,
                            comment='fuzz back-travel (hook)')

            # retract: pull back before the non-extruding legs so they don't
            # ooze/string
            if params.retract > 0:
                travel_e = travel_e - params.retract
                em.move('G1', last_x, last_y, split['z'], travel_e, params.retract_feedrate,
                        comment='fuzz retract')

            # leg 2: continue outward, dry (only if there's a dry length to
            # add and back-travel isn't in play)
            if not back_travel_active and dry_len > 1e-9:
                last_x = split['x'] + onx * chosen_total
                last_y = split['y'] + ony * chosen_total
                em.move('G1', last_x, last_y, split['z'], travel_e, dry_feedrate, comment='fuzz out (dry)')

            # z-hop: lift clear of the hair before dragging back over it
            return_z = split['z']
            if params.z_hop > 0:
                return_z = split['z'] + params.z_hop
                em.move('G1', last_x, last_y, return_z, travel_e, params.z_hop_feedrate,
                        comment='fuzz z-hop up')

            # leg 3: return to the split point, whole way in one move since
            # both outward legs were collinear. --return-extrude was
            # already spent during back-travel above (if that happened),
            # so it isn't applied a second time here.
            leftover_extrude = 0.0 if return_extrude_used else params.return_extrude
            back_e = travel_e + leftover_extrude
            return_feedrate = params.feedrate if leftover_extrude > 0 else dry_feedrate
            em.move('G1', split['x'], split['y'], return_z, back_e, return_feedrate, comment='fuzz back')

            if params.z_hop > 0:
                em.move('G1', split['x'], split['y'], split['z'], back_e, params.z_hop_feedrate,
                        comment='fuzz z-hop down')

            # unretract: restore what was pulled back (plus a little extra
            # to rebuild melt pressure, if requested), before resuming the
            # wall path
            if params.retract > 0:
                back_e = back_e + params.retract + params.extra_restart
                em.move('G1', split['x'], split['y'], split['z'], back_e, params.retract_feedrate,
                        comment='fuzz unretract')

            if params.fan_boost:
                em.lines.append(fan_restore_line(current_fan_speed))

            # hide the extra filament from the absolute-E timeline so the
            # untouched remainder of the file is still correct
            em.reset_e_register(split['e'])

            loops_added += 1
            dist_since_last = 0.0
            # NB: deliberately NOT reassigning `a` to `split` here -- t is
            # computed relative to the original edge (a, b, seg_len), so
            # interpolation must keep using that same fixed frame for every
            # split on this edge, or successive loops drift further apart
            # than --spacing actually asks for.

        # finish the (remainder of the) edge normally
        em.move('G1', b['x'], b['y'], b['z'], b['e'], b['f'])
        prev = b

    fuzzify_contour.stats_loops += loops_added
    fuzzify_contour.stats_skipped += loops_skipped
    fuzzify_contour.stats_contours += 1
    return em.lines


def _fuzzify_hair_plugs(contour, params, mode_xyz_abs, mode_e_abs, rng,
                         grid=None, own_contour_id=None, current_fan_speed=0.0):
    """Prints the wall exactly as sliced (no interruptions), records where
    each hair would go along the way, then applies all of them as a
    separate pass once the wall is done -- travel to each spot, grow the
    hair, travel to the next. Stays retracted between consecutive hairs
    rather than unretracting and immediately retracting again."""

    geom_points = [p for p in contour if not p.get('marker')]

    if len(geom_points) < 2:
        return [p['raw_line'] for p in contour if p.get('raw_line')]

    fuzzy_map = get_fuzzy_map(params)

    total_len = sum(dist2d(geom_points[i], geom_points[i + 1]) for i in range(len(geom_points) - 1))
    if total_len < params.min_contour_length:
        return [p['raw_line'] for p in contour if p.get('raw_line')]

    ccw = signed_area2(geom_points) > 0
    cx = sum(p['x'] for p in geom_points) / len(geom_points)
    cy = sum(p['y'] for p in geom_points) / len(geom_points)

    em = Emitter(geom_points[0], mode_xyz_abs, mode_e_abs)

    spacing = max(params.spacing, 0.1)
    dist_since_last = rng.uniform(0, spacing) if params.random_phase else spacing / 2.0

    # -- pass 1: replay the wall untouched, recording candidate hair spots --
    candidates = []   # (split_point, onx, ony, chosen_total, extrude_ratio)
    prev = None
    for entry in contour:
        if entry.get('marker'):
            em.lines.append(entry['raw_line'])
            continue
        if prev is None:
            prev = entry
            continue

        a, b = prev, entry
        seg_len = dist2d(a, b)
        if seg_len <= 1e-9:
            prev = b
            continue

        edge_tool = b.get('tool')
        edge_ok = (not fuzzy_map) or (edge_tool in fuzzy_map)

        if edge_ok:
            tx, ty = (b['x'] - a['x']) / seg_len, (b['y'] - a['y']) / seg_len
            nx, ny = (ty, -tx) if ccw else (-ty, tx)

            walked = 0.0
            while True:
                remaining = seg_len - walked
                need = spacing - dist_since_last
                if need > remaining:
                    dist_since_last += remaining
                    break
                walked += need
                t = walked / seg_len
                split = lerp_point(a, b, t)

                snx, sny = nx, ny
                if (snx * (split['x'] - cx) + sny * (split['y'] - cy)) < 0:
                    snx, sny = -snx, -sny

                back_travel_active = params.back_travel > 0
                base_total = params.length if back_travel_active else (params.length + params.dry_length)
                jittered_total = base_total * (1.0 + rng.uniform(-params.length_jitter, params.length_jitter))
                jittered_total = max(jittered_total, 0.0)
                if back_travel_active:
                    extrude_ratio = 1.0
                else:
                    extrude_ratio = (params.length / base_total) if base_total > 1e-9 else 1.0

                angle = math.radians(rng.uniform(-params.angle_jitter, params.angle_jitter))
                onx = snx * math.cos(angle) - sny * math.sin(angle)
                ony = snx * math.sin(angle) + sny * math.cos(angle)

                chosen_total = None
                if grid is None:
                    chosen_total = jittered_total
                else:
                    for frac in SHRINK_STEPS:
                        cand_total = jittered_total * frac
                        tip = (split['x'] + onx * cand_total, split['y'] + ony * cand_total)
                        if not loop_collides(split, tip, grid, own_contour_id, params.collision_margin):
                            chosen_total = cand_total
                            break

                if chosen_total is not None:
                    candidates.append((split, onx, ony, chosen_total, extrude_ratio))
                else:
                    fuzzify_contour.stats_skipped += 1
                dist_since_last = 0.0
        else:
            dist_since_last = min(dist_since_last + seg_len, spacing)

        # wall prints exactly as sliced -- no interruption, ever
        em.move('G1', b['x'], b['y'], b['z'], b['e'], b['f'])
        prev = b

    # -- pass 2: apply every recorded hair as its own travel-there, grow,
    #    travel-away sequence, staying retracted between consecutive hairs --
    is_retracted = False
    dry_feedrate = params.dry_feedrate or params.feedrate
    z_up = geom_points[0]['z'] + params.z_hop if params.z_hop > 0 else None

    def travel_to(x, y, e_now, z_now):
        nonlocal is_retracted
        if params.retract > 0 and not is_retracted:
            e_now = e_now - params.retract
            em.move('G1', em.last['x'], em.last['y'], z_now, e_now, params.retract_feedrate,
                    comment='fuzz retract (travel)')
            is_retracted = True
        if params.z_hop > 0:
            em.move('G1', em.last['x'], em.last['y'], z_up, e_now, params.z_hop_feedrate,
                    comment='fuzz z-hop up')
            em.move('G1', x, y, z_up, e_now, dry_feedrate, comment='fuzz travel')
            em.move('G1', x, y, z_now, e_now, params.z_hop_feedrate, comment='fuzz z-hop down')
        else:
            em.move('G1', x, y, z_now, e_now, dry_feedrate, comment='fuzz travel')
        return e_now

    for (split, onx, ony, chosen_total, extrude_ratio) in candidates:
        # wall overlap: head straight for the (possibly inward-shifted)
        # root instead of the wall edge -- see _fuzzify_wall_interrupt for
        # the rationale. Outward legs still measure from the edge (split).
        overlap_shift = params.wall_thickness * (params.wall_overlap / 100.0)
        root_x = split['x'] - onx * overlap_shift
        root_y = split['y'] - ony * overlap_shift

        e_now = travel_to(root_x, root_y, em.last['e'], split['z'])

        if params.retract > 0 and is_retracted:
            e_now = e_now + params.retract + params.extra_restart
            em.move('G1', root_x, root_y, split['z'], e_now, params.retract_feedrate,
                    comment='fuzz unretract')
            is_retracted = False

        if params.fan_boost:
            em.lines.append("M106 S255 ; hairy_walls: fan boost\n")

        extrude_len = chosen_total * extrude_ratio
        dry_len = chosen_total - extrude_len

        if params.root_extrude > 0:
            e_now = e_now + params.root_extrude
            em.move('G1', root_x, root_y, split['z'], e_now, params.feedrate, comment='fuzz root')

        ext_x = split['x'] + onx * extrude_len
        ext_y = split['y'] + ony * extrude_len
        e_now = e_now + params.extrude
        em.move('G1', ext_x, ext_y, split['z'], e_now, params.feedrate, comment='fuzz out (extrude)')

        last_x, last_y = ext_x, ext_y
        return_extrude_used = False

        # back-travel: drag the tip backward toward the part, before
        # retracting -- curls the still-soft tip into a hook shape.
        # Disables the dry leg for this loop when active. --return-extrude,
        # if set, is extruded DURING this backward leg (over just the
        # back-travel distance) instead of over the whole return trip.
        if params.back_travel > 0:
            last_x = ext_x - onx * params.back_travel
            last_y = ext_y - ony * params.back_travel
            if params.return_extrude > 0:
                e_now = e_now + params.return_extrude
                em.move('G1', last_x, last_y, split['z'], e_now, params.feedrate,
                        comment='fuzz back-travel (hook, extruding)')
                return_extrude_used = True
            else:
                em.move('G1', last_x, last_y, split['z'], e_now, dry_feedrate,
                        comment='fuzz back-travel (hook)')

        if params.retract > 0:
            e_now = e_now - params.retract
            em.move('G1', last_x, last_y, split['z'], e_now, params.retract_feedrate, comment='fuzz retract')
            is_retracted = True

        if params.back_travel <= 0 and dry_len > 1e-9:
            last_x = split['x'] + onx * chosen_total
            last_y = split['y'] + ony * chosen_total
            em.move('G1', last_x, last_y, split['z'], e_now, dry_feedrate, comment='fuzz out (dry)')

        return_z = split['z']
        if params.z_hop > 0:
            return_z = split['z'] + params.z_hop
            em.move('G1', last_x, last_y, return_z, e_now, params.z_hop_feedrate, comment='fuzz z-hop up')

        # --return-extrude was already spent during back-travel above (if
        # that happened), so it isn't applied a second time here.
        leftover_extrude = 0.0 if return_extrude_used else params.return_extrude
        e_now = e_now + leftover_extrude
        return_feedrate = params.feedrate if leftover_extrude > 0 else dry_feedrate
        em.move('G1', split['x'], split['y'], return_z, e_now, return_feedrate, comment='fuzz back')

        if params.z_hop > 0:
            em.move('G1', split['x'], split['y'], split['z'], e_now, params.z_hop_feedrate,
                    comment='fuzz z-hop down')

        if params.fan_boost:
            em.lines.append(fan_restore_line(current_fan_speed))

        fuzzify_contour.stats_loops += 1

    # restore normal (unretracted) state before whatever comes next in the
    # file, and hide all the hair filament from the absolute-E timeline
    if is_retracted and params.retract > 0:
        e_now = em.last['e'] + params.retract + params.extra_restart
        em.move('G1', em.last['x'], em.last['y'], em.last['z'], e_now, params.retract_feedrate,
                comment='fuzz unretract (final)')

    em.reset_e_register(geom_points[-1]['e'])

    fuzzify_contour.stats_contours += 1
    return em.lines


fuzzify_contour.stats_loops = 0
fuzzify_contour.stats_skipped = 0
fuzzify_contour.stats_contours = 0


# --------------------------------------------------------------------------
# collision checking (opt-in, --avoid-collisions)
# --------------------------------------------------------------------------

def point_seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def seg_crosses(p1, p2, p3, p4, eps=1e-7):
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    d1, d2 = cross(p3, p4, p1), cross(p3, p4, p2)
    d3, d4 = cross(p1, p2, p3), cross(p1, p2, p4)
    return (((d1 > eps and d2 < -eps) or (d1 < -eps and d2 > eps)) and
            ((d3 > eps and d4 < -eps) or (d3 < -eps and d4 > eps)))


def seg_seg_dist(p1, p2, p3, p4):
    if seg_crosses(p1, p2, p3, p4):
        return 0.0
    return min(point_seg_dist(p1, p3, p4), point_seg_dist(p2, p3, p4),
               point_seg_dist(p3, p1, p2), point_seg_dist(p4, p1, p2))


# fractions of the requested loop length to try, in order, before giving up
SHRINK_STEPS = (1.0, 0.75, 0.5, 0.3)


class SegGrid:
    """Uniform grid spatial index over 2D line segments, for fast
    'what's near this candidate fuzz loop' queries on a busy layer."""

    def __init__(self, cell_size):
        self.cell_size = max(cell_size, 0.5)
        self.cells = {}
        self.overflow = []   # segments whose bbox is too large to bin sanely

    def _cell_range(self, x0, y0, x1, y1):
        cs = self.cell_size
        gx0, gx1 = int(math.floor(min(x0, x1) / cs)), int(math.floor(max(x0, x1) / cs))
        gy0, gy1 = int(math.floor(min(y0, y1) / cs)), int(math.floor(max(y0, y1) / cs))
        return gx0, gx1, gy0, gy1

    def add(self, x1, y1, x2, y2, contour_id):
        gx0, gx1, gy0, gy1 = self._cell_range(x1, y1, x2, y2)
        if (gx1 - gx0 + 1) * (gy1 - gy0 + 1) > 400:
            self.overflow.append((x1, y1, x2, y2, contour_id))
            return
        seg = (x1, y1, x2, y2, contour_id)
        for gx in range(gx0, gx1 + 1):
            for gy in range(gy0, gy1 + 1):
                self.cells.setdefault((gx, gy), []).append(seg)

    def query(self, x1, y1, x2, y2, margin, exclude_contour_id):
        gx0, gx1, gy0, gy1 = self._cell_range(x1 - margin, y1 - margin, x2 + margin, y2 + margin)
        seen = set()
        result = []
        for gx in range(gx0, gx1 + 1):
            for gy in range(gy0, gy1 + 1):
                for seg in self.cells.get((gx, gy), ()):
                    if seg[4] == exclude_contour_id or id(seg) in seen:
                        continue
                    seen.add(id(seg))
                    result.append(seg)
        for seg in self.overflow:
            if seg[4] != exclude_contour_id and id(seg) not in seen:
                seen.add(id(seg))
                result.append(seg)
        return result


def loop_collides(split, tip, grid, own_contour_id, margin):
    p1, p2 = (split['x'], split['y']), (tip[0], tip[1])
    for (sx1, sy1, sx2, sy2, _cid) in grid.query(p1[0], p1[1], p2[0], p2[1], margin, own_contour_id):
        if seg_seg_dist(p1, p2, (sx1, sy1), (sx2, sy2)) <= margin:
            return True
    return False


def get_fuzzy_map(params):
    """Combine up to 4 (fuzzy tool -> remap target) slots into one dict.
    An empty dict means "no fuzzy tool filtering configured" -- fuzz the
    whole model, matching the old default when nothing was set."""
    m = {}
    for i in (1, 2, 3, 4):
        tool = getattr(params, f'fuzzy_tool_{i}')
        if tool is not None:
            m[tool] = getattr(params, f'remap_tool_{i}_to')
    return m


def maybe_remap_tool_line(line, tool_num, fuzzy_map):
    """If this T-command activates one of the configured fuzzy tools and a
    real remap target was given for that slot, rewrite it to that real
    tool number so the printer never sees a tool it doesn't have.
    Everything else about the line (indentation, trailing comment) is
    preserved."""
    remap_to = fuzzy_map.get(tool_num)
    if remap_to is not None:
        return TOOL_LINE_RE.sub(lambda m: m.group(1) + str(remap_to), line, count=1)
    return line


# --------------------------------------------------------------------------
# streaming gcode parser / rewriter
# --------------------------------------------------------------------------

def process(lines, params):
    rng = random.Random(params.seed)
    fuzzy_map = get_fuzzy_map(params)

    out = []
    mode_xyz_abs = True
    mode_e_abs = True          # M82 is the printer power-on default
    cur = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'e': 0.0, 'f': 0.0}
    in_outer_wall = False
    z_in_range = True
    current_tool = 0
    current_fan_speed = 0.0
    contour = []

    def flush():
        nonlocal contour
        if not contour:
            return
        if z_in_range:
            out.extend(fuzzify_contour(contour, params, mode_xyz_abs, mode_e_abs, rng,
                                        current_fan_speed=current_fan_speed))
        else:
            out.extend(p['raw_line'] for p in contour if p.get('raw_line'))
        contour = []

    for line in lines:
        stripped = line.strip()

        m = TYPE_RE.match(stripped)
        if m:
            flush()
            t = m.group(1).lower()
            in_outer_wall = any(k in t for k in OUTER_WALL_KEYWORDS)
            out.append(line)
            continue

        if LAYER_RE.match(stripped):
            flush()
            out.append(line)
            continue

        if FAN_ON_RE.match(stripped):
            flush()
            sm = FAN_S_RE.search(stripped)
            current_fan_speed = float(sm.group(1)) if sm else 255.0
            out.append(line)
            continue
        if FAN_OFF_RE.match(stripped):
            flush()
            current_fan_speed = 0.0
            out.append(line)
            continue

        tmatch = TOOL_RE.match(stripped)
        if tmatch:
            # deliberately NOT flushing here: a tool change mid-wall should
            # not break the contour's geometry, only tag which points were
            # drawn with which tool (see 'tool' field below). But if a
            # contour is already being buffered, the T-command line itself
            # must be spliced into that buffer as a marker -- appending it
            # straight to `out` would hoist it in front of geometry that's
            # still sitting in the (not yet flushed) contour, firing the
            # real tool change far too early.
            current_tool = int(tmatch.group(1))
            emit_line = maybe_remap_tool_line(line, current_tool, fuzzy_map)
            if contour:
                contour.append({'marker': True, 'raw_line': emit_line})
            else:
                out.append(emit_line)
            continue

        up = stripped.upper()
        if up.startswith('M82'):
            flush(); mode_e_abs = True; out.append(line); continue
        if up.startswith('M83'):
            flush(); mode_e_abs = False; out.append(line); continue
        if up.startswith('G90'):
            flush(); mode_xyz_abs = True; out.append(line); continue
        if up.startswith('G91'):
            flush(); mode_xyz_abs = False; out.append(line); continue
        if up.startswith('G92'):
            coords = {k.upper(): float(v) for k, v in COORD_RE.findall(stripped)}
            if 'E' in coords:
                cur['e'] = coords['E']
            flush()
            out.append(line)
            continue

        arc_m = ARC_RE.match(stripped)
        if arc_m:
            gcmd = arc_m.group(1).upper()
            coords = {k.upper(): float(v) for k, v in ARC_COORD_RE.findall(stripped)}
            end_x = coords.get('X', cur['x']) if mode_xyz_abs else cur['x'] + coords.get('X', 0.0)
            end_y = coords.get('Y', cur['y']) if mode_xyz_abs else cur['y'] + coords.get('Y', 0.0)
            end_z = coords.get('Z', cur['z']) if mode_xyz_abs else cur['z'] + coords.get('Z', 0.0)
            f_new = coords.get('F', cur['f'])
            e_present = 'E' in coords
            end_e = (coords['E'] if mode_e_abs else cur['e'] + coords['E']) if e_present else cur['e']
            i_off = coords.get('I', 0.0)
            j_off = coords.get('J', 0.0)

            if 'I' not in coords and 'J' not in coords:
                # R-form or malformed arc -- not supported, pass through as-is
                if in_outer_wall:
                    flush()
                out.append(line)
                cur = {'x': end_x, 'y': end_y, 'z': end_z, 'e': end_e, 'f': f_new}
                continue

            samples = arc_points(cur, end_x, end_y, i_off, j_off, clockwise=(gcmd == 'G2'))
            is_extrude_move = e_present and end_e > cur['e']
            z_in_range = params.min_z <= end_z <= params.max_z

            if in_outer_wall and is_extrude_move and z_in_range:
                if not contour:
                    contour.append(dict(cur, raw_line=None, tool=current_tool))
                start_e = cur['e']
                for idx, (sx, sy, t) in enumerate(samples):
                    se = start_e + (end_e - start_e) * t
                    last_sample = (idx == len(samples) - 1)
                    contour.append({'x': sx, 'y': sy, 'z': end_z, 'e': se, 'f': f_new,
                                     'raw_line': line if last_sample else None,
                                     'tool': current_tool})
            else:
                flush()
                out.append(line)

            cur = {'x': end_x, 'y': end_y, 'z': end_z, 'e': end_e, 'f': f_new}
            continue

        mobj = MOVE_RE.match(stripped)
        if not mobj:
            if in_outer_wall:
                flush()
            out.append(line)
            continue

        gcmd = mobj.group(1).upper()
        coords = {k.upper(): float(v) for k, v in COORD_RE.findall(stripped)}
        new = dict(cur)
        has_xy = 'X' in coords or 'Y' in coords

        if 'X' in coords:
            new['x'] = coords['X'] if mode_xyz_abs else cur['x'] + coords['X']
        if 'Y' in coords:
            new['y'] = coords['Y'] if mode_xyz_abs else cur['y'] + coords['Y']
        if 'Z' in coords:
            new['z'] = coords['Z'] if mode_xyz_abs else cur['z'] + coords['Z']
        if 'F' in coords:
            new['f'] = coords['F']
        e_present = 'E' in coords
        if e_present:
            new['e'] = coords['E'] if mode_e_abs else cur['e'] + coords['E']

        if 'Z' in coords or new['z'] != cur['z']:
            z_in_range = (params.min_z <= new['z'] <= params.max_z)

        is_extrude_move = (gcmd == 'G1' and e_present and new['e'] > cur['e'] and has_xy)

        if in_outer_wall and is_extrude_move and z_in_range:
            if not contour:
                contour.append(dict(cur, raw_line=None, tool=current_tool))
            contour.append(dict(new, raw_line=line, tool=current_tool))
        else:
            flush()
            out.append(line)

        cur = new

    flush()
    return out


def split_into_layers(lines):
    """Break the file into layer-sized chunks using the same markers the
    streaming parser already treats as layer boundaries. Falls back to
    treating the whole file as one chunk if no such markers exist at all
    (rare, but keeps --avoid-collisions correct rather than silently
    doing nothing on an unusual file -- just slower on that one file)."""
    boundaries = [i for i, line in enumerate(lines)
                  if LAYER_RE.match(line.strip())]
    if not boundaries:
        return [lines]
    chunks = []
    start = 0
    for b in boundaries:
        if b > start:
            chunks.append(lines[start:b])
        start = b
    chunks.append(lines[start:])
    return chunks


def simulate_chunk(chunk_lines, state, params, grid):
    """Run the same move-tracking logic as process(), but for one layer
    chunk: instead of fuzzifying each wall contour immediately, register
    it (by id) and leave a placeholder in out_template so the whole
    layer's geometry -- including toolpaths that come later in the file --
    is known before any loop is actually committed. Every extruded move
    in the chunk (wall or not) is also recorded into `grid` for collision
    checks. Returns (out_template, wall_contours, next_state)."""

    out_template = []
    wall_contours = {}
    contour_fan_speed = {}
    next_id = [0]
    fuzzy_map = get_fuzzy_map(params)

    mode_xyz_abs = state['mode_xyz_abs']
    mode_e_abs = state['mode_e_abs']
    cur = dict(state['cur'])
    current_tool = state['tool']
    current_fan_speed = state.get('fan_speed', 0.0)
    in_outer_wall = False
    z_in_range = True
    contour = []
    contour_id = [None]

    def start_contour_if_needed():
        if not contour:
            contour_id[0] = next_id[0]
            next_id[0] += 1

    def flush():
        nonlocal contour
        if not contour:
            return
        cid = contour_id[0]
        wall_contours[cid] = contour
        contour_fan_speed[cid] = current_fan_speed
        out_template.append(('contour', cid))
        contour = []
        contour_id[0] = None

    def record_segment(x1, y1, x2, y2, cid):
        if x1 == x2 and y1 == y2:
            return
        grid.add(x1, y1, x2, y2, cid if cid is not None else -1)

    for line in chunk_lines:
        stripped = line.strip()

        m = TYPE_RE.match(stripped)
        if m:
            flush()
            t = m.group(1).lower()
            in_outer_wall = any(k in t for k in OUTER_WALL_KEYWORDS)
            out_template.append(('raw', line))
            continue

        if FAN_ON_RE.match(stripped):
            flush()
            sm = FAN_S_RE.search(stripped)
            current_fan_speed = float(sm.group(1)) if sm else 255.0
            out_template.append(('raw', line))
            continue
        if FAN_OFF_RE.match(stripped):
            flush()
            current_fan_speed = 0.0
            out_template.append(('raw', line))
            continue

        tmatch = TOOL_RE.match(stripped)
        if tmatch:
            current_tool = int(tmatch.group(1))
            emit_line = maybe_remap_tool_line(line, current_tool, fuzzy_map)
            if contour:
                contour.append({'marker': True, 'raw_line': emit_line})
            else:
                out_template.append(('raw', emit_line))
            continue

        up = stripped.upper()
        if up.startswith('M82'):
            flush(); mode_e_abs = True; out_template.append(('raw', line)); continue
        if up.startswith('M83'):
            flush(); mode_e_abs = False; out_template.append(('raw', line)); continue
        if up.startswith('G90'):
            flush(); mode_xyz_abs = True; out_template.append(('raw', line)); continue
        if up.startswith('G91'):
            flush(); mode_xyz_abs = False; out_template.append(('raw', line)); continue
        if up.startswith('G92'):
            coords = {k.upper(): float(v) for k, v in COORD_RE.findall(stripped)}
            if 'E' in coords:
                cur['e'] = coords['E']
            flush()
            out_template.append(('raw', line))
            continue

        arc_m = ARC_RE.match(stripped)
        if arc_m:
            gcmd = arc_m.group(1).upper()
            coords = {k.upper(): float(v) for k, v in ARC_COORD_RE.findall(stripped)}
            end_x = coords.get('X', cur['x']) if mode_xyz_abs else cur['x'] + coords.get('X', 0.0)
            end_y = coords.get('Y', cur['y']) if mode_xyz_abs else cur['y'] + coords.get('Y', 0.0)
            end_z = coords.get('Z', cur['z']) if mode_xyz_abs else cur['z'] + coords.get('Z', 0.0)
            f_new = coords.get('F', cur['f'])
            e_present = 'E' in coords
            end_e = (coords['E'] if mode_e_abs else cur['e'] + coords['E']) if e_present else cur['e']
            i_off = coords.get('I', 0.0)
            j_off = coords.get('J', 0.0)

            if 'I' not in coords and 'J' not in coords:
                if in_outer_wall:
                    flush()
                out_template.append(('raw', line))
                cur = {'x': end_x, 'y': end_y, 'z': end_z, 'e': end_e, 'f': f_new}
                continue

            samples = arc_points(cur, end_x, end_y, i_off, j_off, clockwise=(gcmd == 'G2'))
            is_extrude_move = e_present and end_e > cur['e']
            z_in_range = params.min_z <= end_z <= params.max_z

            if in_outer_wall and is_extrude_move and z_in_range:
                start_contour_if_needed()
                if not contour:
                    contour.append(dict(cur, raw_line=None, tool=current_tool))
                start_e = cur['e']
                px, py = cur['x'], cur['y']
                for idx, (sx, sy, t) in enumerate(samples):
                    se = start_e + (end_e - start_e) * t
                    last_sample = (idx == len(samples) - 1)
                    contour.append({'x': sx, 'y': sy, 'z': end_z, 'e': se, 'f': f_new,
                                     'raw_line': line if last_sample else None,
                                     'tool': current_tool})
                    record_segment(px, py, sx, sy, contour_id[0])
                    px, py = sx, sy
            else:
                flush()
                out_template.append(('raw', line))
                if is_extrude_move and z_in_range:
                    px, py = cur['x'], cur['y']
                    for (sx, sy, t) in arc_points(cur, end_x, end_y, i_off, j_off, clockwise=(gcmd == 'G2')):
                        record_segment(px, py, sx, sy, None)
                        px, py = sx, sy

            cur = {'x': end_x, 'y': end_y, 'z': end_z, 'e': end_e, 'f': f_new}
            continue

        mobj = MOVE_RE.match(stripped)
        if not mobj:
            if in_outer_wall:
                flush()
            out_template.append(('raw', line))
            continue

        gcmd = mobj.group(1).upper()
        coords = {k.upper(): float(v) for k, v in COORD_RE.findall(stripped)}
        new = dict(cur)
        has_xy = 'X' in coords or 'Y' in coords

        if 'X' in coords:
            new['x'] = coords['X'] if mode_xyz_abs else cur['x'] + coords['X']
        if 'Y' in coords:
            new['y'] = coords['Y'] if mode_xyz_abs else cur['y'] + coords['Y']
        if 'Z' in coords:
            new['z'] = coords['Z'] if mode_xyz_abs else cur['z'] + coords['Z']
        if 'F' in coords:
            new['f'] = coords['F']
        e_present = 'E' in coords
        if e_present:
            new['e'] = coords['E'] if mode_e_abs else cur['e'] + coords['E']

        if 'Z' in coords or new['z'] != cur['z']:
            z_in_range = (params.min_z <= new['z'] <= params.max_z)

        is_extrude_move = (gcmd == 'G1' and e_present and new['e'] > cur['e'] and has_xy)

        if in_outer_wall and is_extrude_move and z_in_range:
            start_contour_if_needed()
            if not contour:
                contour.append(dict(cur, raw_line=None, tool=current_tool))
            contour.append(dict(new, raw_line=line, tool=current_tool))
            record_segment(cur['x'], cur['y'], new['x'], new['y'], contour_id[0])
        else:
            flush()
            out_template.append(('raw', line))
            if is_extrude_move and z_in_range:
                record_segment(cur['x'], cur['y'], new['x'], new['y'], None)

        cur = new

    flush()

    new_state = {'mode_xyz_abs': mode_xyz_abs, 'mode_e_abs': mode_e_abs,
                 'cur': cur, 'tool': current_tool, 'fan_speed': current_fan_speed}
    return out_template, wall_contours, contour_fan_speed, new_state


def process_collision_aware(lines, params):
    rng = random.Random(params.seed)
    cell_size = max(params.length * 2.0, 2.0)

    state = {'mode_xyz_abs': True, 'mode_e_abs': True,
             'cur': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'e': 0.0, 'f': 0.0}, 'tool': 0, 'fan_speed': 0.0}

    out = []
    for chunk in split_into_layers(lines):
        grid = SegGrid(cell_size)
        out_template, wall_contours, contour_fan_speed, state = simulate_chunk(chunk, state, params, grid)

        fuzzed = {}
        for cid, contour in wall_contours.items():
            fuzzed[cid] = fuzzify_contour(contour, params, state['mode_xyz_abs'], state['mode_e_abs'],
                                           rng, grid=grid, own_contour_id=cid,
                                           current_fan_speed=contour_fan_speed.get(cid, 0.0))

        for kind, payload in out_template:
            if kind == 'raw':
                out.append(payload)
            else:
                out.extend(fuzzed[payload])

    return out


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def build_arg_parser():
    p = argparse.ArgumentParser(
        prog='hairy_walls.py',
        description="Hairy Walls -- add outward filament fuzz/hair loops along outer wall perimeters.")
    p.add_argument('--version', action='version', version=f"hairy_walls.py {__version__}")
    p.add_argument('input', help="source .gcode file, or a Bambu .gcode.3mf sliced plate")
    p.add_argument('output', help="destination .gcode file, or .gcode.3mf (only when the "
                                   "input is also a .gcode.3mf -- see gcode3mf.py)")
    p.add_argument('--plate', default=None,
                    help="which plate to use inside a multi-plate .gcode.3mf -- a number "
                         "(1, 2, ...) or the exact archive path (e.g. "
                         "Metadata/plate_2.gcode). Only needed if the archive has more "
                         "than one plate; ignored for plain .gcode files")

    p.add_argument('--mode', choices=['wall-interrupt', 'hair-plugs'], default='wall-interrupt',
                    help="wall-interrupt (default): pause the wall path at each spot and "
                         "grow the hair in place, then resume. "
                         "hair-plugs: print each wall exactly as sliced first, then go "
                         "back afterward and apply every hair for that wall as a "
                         "separate pass")
    p.add_argument('--z-hop', type=float, default=1.0,
                    help="mm to lift Z before traveling back from the tip of a hair to "
                         "the wall (and, in hair-plugs mode, between hairs too) so the "
                         "nozzle doesn't drag through what it just printed "
                         "(default 1.0, 0 disables)")
    p.add_argument('--z-hop-feedrate', type=float, default=600,
                    help="mm/min for the Z-hop up/down moves (default 600)")

    p.add_argument('--spacing', type=float, default=4.0,
                    help="mm of wall between the start of each loop (default 4)")
    p.add_argument('--length', type=float, default=2.0,
                    help="mm to travel outward WHILE EXTRUDING, i.e. the length of "
                         "filament that actually gets laid down (default 2.0)")
    p.add_argument('--dry-length', type=float, default=4.0,
                    help="additional mm to keep traveling outward AFTER --length, "
                         "with no extrusion -- lets the very tip of the hair be a "
                         "dry whip rather than filament the whole way out "
                         "(default 4.0, 0 disables)")
    p.add_argument('--back-travel', type=float, default=0.0,
                    help="mm to drag the tip backward, toward the part, right after "
                         "the extrude leg but before retracting -- curls the still-soft "
                         "tip into a hook shape, useful for hook-and-loop fastener "
                         "hairs. A value above 0 disables --dry-length entirely for "
                         "this run: the sequence becomes extrude outward, back-travel, "
                         "retract, return to the wall, un-retract, resume. When active, "
                         "--return-extrude (if set) is extruded DURING this backward "
                         "leg instead of over the whole return trip -- see "
                         "--return-extrude. Intended range is above 0 and below "
                         "--length (default 0, no back-travel)")
    p.add_argument('--wall-thickness', type=float, default=0.0,
                    help="mm, the reference thickness --wall-overlap is a percentage "
                         "of. Set this to your actual wall thickness (line width x "
                         "wall count) to make --wall-overlap meaningful. Only used in "
                         "--mode hair-plugs (see --wall-overlap)")
    p.add_argument('--wall-overlap', type=float, default=0.0,
                    help="0-100. Only used in --mode hair-plugs, ignored in "
                         "wall-interrupt mode (there, the hair always starts exactly "
                         "where the wall path was paused -- already the equivalent of "
                         "100%% overlap, with no separate travel needed to reach it). "
                         "In hair-plugs mode, hairs are approached from outside via a "
                         "travel move, so this controls how far inward from the wall's "
                         "outer edge that travel digs before starting the hair, as a "
                         "percent of --wall-thickness: 0%% roots it right at the edge "
                         "(default), 100%% roots it a full wall-thickness inward so the "
                         "base is embedded in the wall itself. The outward "
                         "leg/dry-length still measure from the edge, so this only "
                         "changes the base, not the visible length")
    p.add_argument('--extrude', type=float, default=0.30,
                    help="mm of filament (E units) to push out over --length of "
                         "outward travel (default 0.30)")
    p.add_argument('--root-extrude', type=float, default=0.0,
                    help="mm of filament to push out AT the wall, with no travel, "
                         "before heading outward -- gives the hair a small anchor "
                         "blob at its base (default 0, no root blob)")
    p.add_argument('--return-extrude', type=float, default=0.0,
                    help="extra E to push during the return-to-wall travel, e.g. for a "
                         "slightly thicker/blobby tip (default 0 -- return is a dry "
                         "move unless you set this). With --back-travel 0 (default) "
                         "this extrudes over the WHOLE return trip, tip to wall. With "
                         "--back-travel above 0, it instead extrudes only over that "
                         "back-travel distance, right at the tip -- the remainder of "
                         "the trip back to the wall stays a dry, retracted move")
    p.add_argument('--retract', type=float, default=0.4,
                    help="mm to retract right after the outward extrude leg, before "
                         "the dry travel and return -- cuts stringing/oozing on the "
                         "parts of the loop that aren't meant to extrude. "
                         "Automatically un-retracted by the same amount once back at "
                         "the wall, before resuming the path (default 0.4, 0 disables)")
    p.add_argument('--retract-feedrate', type=float, default=2100,
                    help="mm/min for the retract and un-retract moves (default 2100)")
    p.add_argument('--extra-restart', type=float, default=0.02,
                    help="extra mm pushed during the un-retract, on top of exactly "
                         "restoring --retract -- real nozzles need a bit more than "
                         "the exact retracted distance back to rebuild melt pressure, "
                         "so if the wall looks under-extruded right after each hair, "
                         "raise this a little (try 0.02-0.1mm) instead of dropping "
                         "--retract (default 0.02)")
    p.add_argument('--feedrate', type=float, default=1200,
                    help="mm/min for any move that's actively extruding -- the "
                         "outward --length travel, and the return travel too if "
                         "--return-extrude is set above 0 (default 1200)")
    p.add_argument('--dry-feedrate', type=float, default=2400,
                    help="mm/min for any move that's NOT extruding -- the outward "
                         "--dry-length travel, and the return travel when "
                         "--return-extrude is 0 (default 2400)")

    p.add_argument('--length-jitter', type=float, default=0.0,
                    help="fractional random variation in loop length, 0-1 (default 0)")
    p.add_argument('--angle-jitter', type=float, default=0.0,
                    help="+/- degrees of random rotation off the pure perpendicular, "
                         "for a less uniform 'hairy' look (default 0)")
    p.add_argument('--random-phase', action=argparse.BooleanOptionalAction, default=True,
                    help="randomize where the first loop of each contour starts, "
                         "instead of always spacing/2 (default on; use "
                         "--no-random-phase to turn off)")
    p.add_argument('--fan-boost', action=argparse.BooleanOptionalAction, default=True,
                    help="max the part-cooling fan (M106 S255) right before each hair "
                         "forms, then set it back to whatever it was (tracked from the "
                         "M106/M107 commands already in the file) right after "
                         "(default on; use --no-fan-boost to turn off)")
    p.add_argument('--seed', type=int, default=0, help="random seed (default 0)")

    p.add_argument('--min-contour-length', type=float, default=1.0,
                    help="skip outer-wall contours shorter than this, in mm "
                         "(default 1 - avoids tiny holes/text getting loops)")
    p.add_argument('--min-z', type=float, default=0.0,
                    help="only add loops at/above this Z height, mm (default 0)")
    p.add_argument('--max-z', type=float, default=float('inf'),
                    help="only add loops at/below this Z height, mm (default: no limit)")
    p.add_argument('--fuzzy-tool-1', type=int, default=None,
                    help="1st of up to 4 independent fuzzy-tool slots -- only add loops "
                         "while this tool (T0, T1, ...) is active. Pair with "
                         "multi-material painting in your slicer to fuzz only painted "
                         "regions. If NONE of the 4 slots are set, loops apply to the "
                         "whole model")
    p.add_argument('--remap-tool-1-to', type=int, default=None,
                    help="rewrite every T-command that activates --fuzzy-tool-1 to this "
                         "real tool number instead -- e.g. paint to an unused virtual "
                         "T4 in your slicer as a marker, then have it actually print "
                         "with a real toolhead. Requires --fuzzy-tool-1 to also be set")
    p.add_argument('--fuzzy-tool-2', type=int, default=None, help="2nd fuzzy-tool slot")
    p.add_argument('--remap-tool-2-to', type=int, default=None, help="remap target for slot 2")
    p.add_argument('--fuzzy-tool-3', type=int, default=None, help="3rd fuzzy-tool slot")
    p.add_argument('--remap-tool-3-to', type=int, default=None, help="remap target for slot 3")
    p.add_argument('--fuzzy-tool-4', type=int, default=None, help="4th fuzzy-tool slot")
    p.add_argument('--remap-tool-4-to', type=int, default=None, help="remap target for slot 4")

    p.add_argument('--avoid-collisions', action=argparse.BooleanOptionalAction, default=True,
                    help="check each candidate loop against every other toolpath in "
                         "the same layer (other walls, infill, skin, support, ...) "
                         "and shrink or drop it rather than let it cross one. Slower "
                         "and uses more memory than streaming (buffers a full layer at "
                         "a time) -- default on; use --no-avoid-collisions for the "
                         "faster streaming path if you don't need it")
    p.add_argument('--collision-margin', type=float, default=1.0,
                    help="minimum clearance, in mm, a loop must keep from any other "
                         "toolpath; 0 only rejects an actual crossing (default 1.0, "
                         "only used with --avoid-collisions)")

    return p


def main(argv=None):
    args = build_arg_parser().parse_args(argv)

    for i in (1, 2, 3, 4):
        remap = getattr(args, f'remap_tool_{i}_to')
        tool = getattr(args, f'fuzzy_tool_{i}')
        if remap is not None and tool is None:
            print(f"hairy_walls: --remap-tool-{i}-to requires --fuzzy-tool-{i} to also "
                  f"be set (nothing to remap otherwise)", file=sys.stderr)
            sys.exit(1)

    input_is_3mf = args.input.lower().endswith('.3mf')
    output_is_3mf = args.output.lower().endswith('.3mf')
    if (input_is_3mf or output_is_3mf) and gcode3mf is None:
        print("hairy_walls: gcode3mf.py is required for .3mf input/output but wasn't "
              "found -- make sure it's in the same folder as hairy_walls.py",
              file=sys.stderr)
        sys.exit(1)

    try:
        if gcode3mf is not None:
            lines = gcode3mf.read_lines(args.input, plate=args.plate)
        else:
            with open(args.input, 'r', encoding='utf-8', errors='replace') as f:
                lines = f.readlines()
    except (ValueError, OSError) as e:
        print(f"hairy_walls: couldn't read {args.input}: {e}", file=sys.stderr)
        sys.exit(1)

    if args.avoid_collisions:
        result = process_collision_aware(lines, args)
    else:
        result = process(lines, args)

    try:
        if gcode3mf is not None:
            gcode3mf.write_lines(args.output, result, src=args.input, plate=args.plate)
        else:
            with open(args.output, 'w', encoding='utf-8') as f:
                f.writelines(result)
    except (ValueError, OSError) as e:
        print(f"hairy_walls: couldn't write {args.output}: {e}", file=sys.stderr)
        sys.exit(1)

    if output_is_3mf:
        print("hairy_walls: note -- cached print-time/filament-usage estimates inside "
              "the .3mf project were not recalculated and may now read low, since this "
              "adds material the original slice didn't account for.", file=sys.stderr)

    skip_note = (f", {fuzzify_contour.stats_skipped} dropped (no collision-free length found)"
                 if args.avoid_collisions else "")
    print(f"hairy_walls {__version__}: {fuzzify_contour.stats_loops} loops added across "
          f"{fuzzify_contour.stats_contours} outer-wall contours{skip_note} "
          f"-> {args.output}", file=sys.stderr)


if __name__ == '__main__':
    main()
