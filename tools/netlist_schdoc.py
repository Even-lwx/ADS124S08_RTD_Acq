import argparse
from collections import defaultdict

from inspect_schdoc import records


def xy(p):
    try:
        return int(p["Location.X"]), int(p["Location.Y"])
    except (KeyError, ValueError):
        return None


class DSU:
    def __init__(self):
        self.p = {}

    def find(self, x):
        self.p.setdefault(x, x)
        if self.p[x] != x:
            self.p[x] = self.find(self.p[x])
        return self.p[x]

    def union(self, a, b):
        a, b = self.find(a), self.find(b)
        if a != b:
            self.p[b] = a


def on_segment(p, a, b):
    return ((b[0] - a[0]) * (p[1] - a[1]) == (b[1] - a[1]) * (p[0] - a[0])
            and min(a[0], b[0]) <= p[0] <= max(a[0], b[0])
            and min(a[1], b[1]) <= p[1] <= max(a[1], b[1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    args = ap.parse_args()
    recs = list(records(args.path))
    props = [x[0] for x in recs]

    designators = {}
    for p in props:
        if p.get("RECORD") == "34" and p.get("Name") == "Designator":
            designators[int(p["OwnerIndex"])] = p.get("Text", "?")

    items = defaultdict(list)
    points = set()
    segments = []
    labels_by_name = defaultdict(list)
    dsu = DSU()

    for idx, p in enumerate(props):
        r = p.get("RECORD")
        loc = xy(p)
        if r == "27":
            n = int(p.get("LocationCount", "0"))
            poly = [(int(p[f"X{i}"]), int(p[f"Y{i}"])) for i in range(1, n + 1)]
            for a, b in zip(poly, poly[1:]):
                segments.append((a, b))
                points.update((a, b))
                dsu.union(a, b)
        elif r == "2" and loc:
            owner = int(p.get("OwnerIndex", "-1"))
            ref = designators.get(owner, f"owner#{owner}")
            items[loc].append(f"PIN {ref}.{p.get('Designator')} {p.get('Name')}")
            points.add(loc)
        elif r in ("25", "17") and loc:
            kind = "NET" if r == "25" else "PWR"
            name = p.get("Text", "")
            items[loc].append(f"{kind} {name}")
            labels_by_name[name].append(loc)
            points.add(loc)

    # Any pin/label/wire vertex located on a wire segment belongs to that wire.
    for p in points:
        for a, b in segments:
            if on_segment(p, a, b):
                dsu.union(p, a)

    # Net labels and power ports connect equal names globally.
    for locs in labels_by_name.values():
        for loc in locs[1:]:
            dsu.union(locs[0], loc)

    nets = defaultdict(list)
    for loc, vals in items.items():
        nets[dsu.find(loc)].extend(vals)

    interesting = ("AIN", "REF", "AVSS", "DGND", "AGND", "GND", "3V3", "A3V3",
                   "VIN", "VOUT", "SPI", "CS", "SCLK", "DIN", "DOUT", "DRDY",
                   "U1.", "U2.", "U3.", "U4.", "U5.", "U6.", "R22.")
    selected = []
    for vals in nets.values():
        if any(any(key in v for key in interesting) for v in vals):
            selected.append(sorted(set(vals)))
    for vals in sorted(selected, key=lambda v: " ".join(v)):
        print(" | ".join(vals))


if __name__ == "__main__":
    main()
