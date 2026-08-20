"""Read/write G-code from either a plain .gcode or a Bambu .gcode.3mf."""
import hashlib
import os
import zipfile


def is_3mf(path):
    return path.lower().endswith('.3mf')


def split_ext(path):
    """Like os.path.splitext but keeps '.gcode.3mf' together."""
    low = path.lower()
    for e in ('.gcode.3mf', '.gcode', '.gco', '.g', '.3mf'):
        if low.endswith(e):
            return path[:-len(e)], path[-len(e):]
    return os.path.splitext(path)


def _member(names, plate=None):
    """Pick which archive member holds the plate G-code. `plate` lets the
    caller disambiguate a multi-plate archive: either the exact member
    path, or a 1-based index into the sorted list of plates. With no
    plates at all this raises; with more than one and no `plate` given,
    it also raises (listing them) rather than silently guessing -- a
    wrong silent guess means processing the wrong plate with no signal
    anything was off."""
    hits = sorted(n for n in names if n.endswith('.gcode'))
    if not hits:
        raise ValueError("no .gcode inside archive -- is this a sliced plate file?")

    if plate is not None:
        if plate in hits:
            return plate
        try:
            idx = int(plate)
        except (TypeError, ValueError):
            idx = None
        if idx is not None and 1 <= idx <= len(hits):
            return hits[idx - 1]
        listing = '\n'.join(f"  {i + 1}. {n}" for i, n in enumerate(hits))
        raise ValueError(f"--plate {plate!r} doesn't match any plate in this "
                          f"archive. Available plates:\n{listing}")

    if len(hits) > 1:
        listing = '\n'.join(f"  {i + 1}. {n}" for i, n in enumerate(hits))
        raise ValueError(f"This archive has {len(hits)} plates -- pick one with "
                          f"--plate (a number or the full path):\n{listing}")

    return hits[0]


def read_lines(path, plate=None):
    if not is_3mf(path):
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            return f.readlines()
    with zipfile.ZipFile(path) as z:
        data = z.read(_member(z.namelist(), plate))
    # Normalize line endings the same way Python's universal-newline text
    # mode does for a plain .gcode file (translate \r\n and lone \r to
    # \n). Without this, CRLF-sourced archive content would keep \r\n on
    # every untouched pass-through line while every newly generated line
    # uses plain \n, mixing both in the same output file.
    text = data.decode('utf-8', 'replace').replace('\r\n', '\n').replace('\r', '\n')
    return text.splitlines(keepends=True)


def write_lines(path, lines, src=None, plate=None):
    if not is_3mf(path):
        with open(path, 'w', encoding='utf-8') as f:
            f.writelines(lines)
        return

    if src and is_3mf(src) and os.path.exists(src):
        archive = src
    elif os.path.exists(path):
        # Overwriting a .gcode.3mf in place with no separate source: use
        # the existing output file itself as the template. Safe because
        # the whole archive is read into memory (below) before this
        # function ever opens `path` for writing, so there's no
        # read/write race against the same file.
        archive = path
    else:
        raise ValueError(
            "Can't save as .gcode.3mf when the input is a plain .gcode.\n\n"
            "A sliced plate file also contains plate metadata, thumbnails and the "
            "model, which can only be copied from an existing .gcode.3mf.\n\n"
            "Either pick a .gcode.3mf as the input, or save the output as .gcode.")

    with zipfile.ZipFile(archive) as z:
        names = z.namelist()
        items = {n: z.read(n) for n in names}

    # NB: pass the same `plate` used for the matching read_lines() call --
    # on a multi-plate archive, picking independently here could silently
    # target a different plate than the one that was actually processed.
    target = _member(names, plate)
    items[target] = ''.join(lines).encode('utf-8')
    md5 = target + '.md5'
    if md5 in items:
        items[md5] = hashlib.md5(items[target]).hexdigest().encode()

    with zipfile.ZipFile(path, 'w', zipfile.ZIP_DEFLATED) as z:
        for n in names:
            z.writestr(n, items[n])
