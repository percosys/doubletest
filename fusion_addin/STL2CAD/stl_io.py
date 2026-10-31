"""Pure-Python binary STL writer (no numpy: Fusion's bundled Python may not have it)."""

import struct


def write_binary_stl(path, coords, indices, scale=1.0):
    """coords: flat x,y,z list; indices: flat triangle vertex indices; scale multiplies coords.

    Fusion's API is in centimetres; pass scale=10 to write millimetres.
    """
    n = len(indices) // 3
    pack = struct.Struct("<12fH").pack
    with open(path, "wb") as fh:
        fh.write(b"stl2cad".ljust(80, b"\0"))
        fh.write(struct.pack("<I", n))
        for t in range(n):
            a, b, c = indices[3 * t], indices[3 * t + 1], indices[3 * t + 2]
            va = [coords[3 * a + k] * scale for k in range(3)]
            vb = [coords[3 * b + k] * scale for k in range(3)]
            vc = [coords[3 * c + k] * scale for k in range(3)]
            fh.write(pack(0.0, 0.0, 0.0, *va, *vb, *vc, 0))
    return n
