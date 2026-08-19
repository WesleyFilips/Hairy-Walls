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


def _member(names):
    hits = [n for n in names if n.endswith('.gcode')]
    if not hits:
        raise ValueError("no .gcode inside archive -- is this a sliced plate file?")
    return hits[0]


def read_lines(path):
    if not is_3mf(path):
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            return f.readlines()
    with zipfile.ZipFile(path) as z:
        data = z.read(_member(z.namelist()))
    return data.decode('utf-8', 'replace').splitlines(keepends=True)


def write_lines(path, lines, src=None):
    if not is_3mf(path):
        with open(path, 'w', encoding='utf-8') as f:
            f.writelines(lines)
        return

    if src and is_3mf(src) and os.path.exists(src):
        archive = src
    elif os.path.exists(path):
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

    target = _member(names)
    items[target] = ''.join(lines).encode('utf-8')
    md5 = target + '.md5'
    if md5 in items:
        items[md5] = hashlib.md5(items[target]).hexdigest().encode()

    with zipfile.ZipFile(path, 'w', zipfile.ZIP_DEFLATED) as z:
        for n in names:
            z.writestr(n, items[n])
