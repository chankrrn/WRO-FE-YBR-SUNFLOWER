"""
Turn color samples into COLOR_SPECS rows for utils/image_color_utils.py - for
ALL the block/line colors at once, so they come out guaranteed not to overlap.

Solving the colors together is the point. Built one at a time, each color's
window only knows its own samples, and red, pink and orange sit next to each
other on the hue circle: widen any one for lighting and it reaches into its
neighbour, and a pixel that matches two colors is a block that flickers
between them. Here every window is first built from its own samples, then
every pair that overlaps is cut apart BETWEEN their samples, on whichever
channel (hue, saturation or value) separates those samples the most. So every
sample still lands in its own color, and no pixel can land in two.

How to use it:
    1. python test_color_picker.py     click each block/line several times;
                                       every click prints an RGB=(...) line
    2. python make_color_spec.py       it asks for RED, GREEN, PINK, ORANGE,
                                       then WHITE (mat) and BLACK (walls):
                                       paste that color's lines and press
                                       Enter on an empty line to move on
                                       (Enter straight away skips a color)
    3. paste the printed rows into COLOR_SPECS

    python make_color_spec.py --save samples.json      keep what you pasted
    python make_color_spec.py --samples samples.json   re-solve it later

Pasted lines can hold one or more of:
    (412,230)  RGB=(177,45,81)  HSV=(174,190,177)    a click from test_color_picker.py
    177, 45, 81                                      a bare RGB triple
    #b12d51                                          a hex color

Useful flags:
    --colors RED,GREEN,PINK,ORANGE    which colors to solve (default: those four)
    --hue-margin 3 --sv-margin 15     how far past the samples each band reaches
                                      before neighbours are cut out of it
"""
import argparse
import json
import math
import re
import sys
from dataclasses import dataclass

import cv2
import numpy as np

from utils.enums import Color
from utils.image_color_utils import COLOR_SPECS, HUE_MAX, HUE_SEAM, build_hsv_range, rgb_to_hue

DEFAULT_COLORS = ("RED", "GREEN", "PINK", "ORANGE")

# Asked for after the colors: things the camera must NEVER call a color. They
# are solved alongside the colors, so every color is cut away from them, but
# no spec is printed for them. Without them nothing stops a color's window
# reaching into near-grey saturations - which is how white read as pink.
AVOID = {
    "WHITE": "the white mat, and glare on anything",
    "BLACK": "the black walls",
}

# Stray values - a click on glare, an edge, or the wrong block - are dropped
# when they sit this many MADs off the rest of their color on hue or
# saturation. Value is left alone: near and far, lit and shaded, it really
# does spread that much. The floors stop a tight color from rejecting a value
# one unit off.
# Red and pink sit ~7 hue units apart, so a hue cut-off has to be well under
# that or a click on the neighbouring block passes as one of ours.
OUTLIER_MADS = 3.5
OUTLIER_FLOOR = {"H": 1.5, "S": 10.0}
OUTLIER_MIN_SAMPLES = 4

# A color whose window still spans this much after outliers are gone is
# almost certainly mixed with something else.
WIDE_HUE = 20
WIDE_SATURATION = 130

# With this many samples or more (a clicked patch is 25), the band is taken
# from the 2nd-98th percentile instead of the extremes, so one stray pixel at
# the edge of a block cannot drag a whole color's window out.
ROBUST_MIN_SAMPLES = 30
ROBUST_PERCENTILE = 2.0

RGB_TAG = re.compile(r"RGB=\((\d+),\s*(\d+),\s*(\d+)\)")
HEX = re.compile(r"#?([0-9a-fA-F]{6})\b")
NUMBER = re.compile(r"\d+")


# ============================================================================
# Parsing
# ============================================================================

def parse_samples(line):
    """
    Every RGB triple in a pasted line - several picker clicks can arrive on one
    line when a terminal joins them up.

    I/O:
        line (str): picker clicks, bare triples or hex colors.
        returns: list[tuple]: (R, G, B) for each color found, possibly empty.
    """
    tagged = RGB_TAG.findall(line)
    if tagged:
        return [tuple(int(c) for c in rgb) for rgb in tagged]
    single = parse_sample(line)
    return [single] if single else []


def parse_sample(line):
    """
    Pull one RGB triple out of a pasted line.

    I/O:
        line (str): a picker click, a bare triple or a hex color.
        returns: tuple|None: (R, G, B), or None if the line holds no color.
    """
    tagged = RGB_TAG.search(line)
    if tagged:
        return tuple(int(c) for c in tagged.groups())
    numbers = NUMBER.findall(line)
    if len(numbers) >= 3:
        rgb = tuple(int(c) for c in numbers[:3])
        if all(c <= 255 for c in rgb):
            return rgb
    hexed = HEX.search(line)
    if hexed:
        h = hexed.group(1)
        return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))
    return None


def rgb_to_hsv(samples):
    pixels = np.uint8([samples])
    return cv2.cvtColor(pixels, cv2.COLOR_RGB2HSV)[0].astype(int)


# ============================================================================
# Range checks - mirrors of what calculate_color_mask actually admits
# ============================================================================

def hue_distance(a, b):
    """Shortest way round OpenCV's 0-179 hue circle between two hues."""
    d = abs(a - b) % HUE_MAX
    return min(d, HUE_MAX - d)


def in_range(hsv, lower, upper):
    """Mirror of calculate_color_mask, including its seam split, for one pixel."""
    h, s, v = hsv
    if not (lower[1] <= s <= upper[1] and lower[2] <= v <= upper[2]):
        return False
    if upper[0] > HUE_SEAM:
        return h >= lower[0] or h <= upper[0] - HUE_SEAM
    return lower[0] <= h <= upper[0]


def hue_set(lower, upper):
    """Every hue a window admits, unwrapping one that crosses the seam."""
    if upper[0] > HUE_SEAM:
        return set(range(lower[0], HUE_MAX)) | set(range(0, upper[0] - HUE_SEAM + 1))
    return set(range(lower[0], upper[0] + 1))


def channel_gaps(a, b):
    """
    How far apart two HSV windows sit on each channel; 0 or less means they meet.

    I/O:
        a, b (tuple): (lower, upper) HSV bounds from build_hsv_range.
        returns: tuple[int, int, int]: hue, saturation and value gaps.
    """
    ha, hb = hue_set(*a), hue_set(*b)
    hue_gap = 0 if ha & hb else min(hue_distance(x, y) for x in ha for y in hb)
    s_gap = max(a[0][1], b[0][1]) - min(a[1][1], b[1][1])
    v_gap = max(a[0][2], b[0][2]) - min(a[1][2], b[1][2])
    return hue_gap, s_gap, v_gap


# ============================================================================
# The joint solve
# ============================================================================

@dataclass
class Window:
    """
    One color's box in HSV while it is being solved.

    Hue is held UNWRAPPED around the samples' circular mean, so a red that
    straddles the 179/0 seam is one continuous interval (e.g. 172..184) rather
    than two. `core_*` is where the samples actually are; the plain bounds are
    the window, which starts as the core plus margins and only ever shrinks -
    and never past the core.
    """
    name: str
    avoid: bool
    samples: list
    hsv: np.ndarray
    center: float
    core_h: list
    core_s: list
    core_v: list
    h: list
    s: list
    v: list


def circular_mean_hue(hues):
    angles = np.asarray(hues) * (2 * math.pi / HUE_MAX)
    mean = math.atan2(np.sin(angles).mean(), np.cos(angles).mean())
    return (mean * HUE_MAX / (2 * math.pi)) % HUE_MAX


def extent(values):
    """Low/high of a set of samples - robust to stray pixels once there are enough."""
    values = np.asarray(values, dtype=float)
    if len(values) >= ROBUST_MIN_SAMPLES:
        low, high = np.percentile(values, [ROBUST_PERCENTILE, 100 - ROBUST_PERCENTILE])
    else:
        low, high = values.min(), values.max()
    return [int(math.floor(low)), int(math.ceil(high))]


def hue_offsets(hues, center):
    """Each hue as a signed offset from `center`, the short way round."""
    return (np.asarray(hues) - center + HUE_MAX / 2) % HUE_MAX - HUE_MAX / 2


def drop_outliers(samples):
    """
    Splits a color's samples into (kept, dropped).

    I/O:
        samples (list): (R, G, B) tuples.
        returns: (list, list) - kept and dropped samples.
    """
    if len(samples) < OUTLIER_MIN_SAMPLES:
        return list(samples), []
    hsv = rgb_to_hsv(samples)
    offsets = hue_offsets(hsv[:, 0], circular_mean_hue(hsv[:, 0]))
    bad = np.zeros(len(samples), dtype=bool)
    for channel, values in (("H", offsets), ("S", hsv[:, 1].astype(float))):
        median = np.median(values)
        mad = max(np.median(np.abs(values - median)), OUTLIER_FLOOR[channel])
        bad |= np.abs(values - median) > OUTLIER_MADS * mad
    kept = [rgb for rgb, b in zip(samples, bad) if not b]
    dropped = [rgb for rgb, b in zip(samples, bad) if b]
    return kept, dropped


def build_window(name, samples, hue_margin, sv_margin):
    hsv = rgb_to_hsv(samples)
    center = circular_mean_hue(hsv[:, 0])
    # Each hue as center + its signed offset, so the interval never wraps.
    offsets = (hsv[:, 0] - center + HUE_MAX / 2) % HUE_MAX - HUE_MAX / 2
    core_h = extent(center + offsets)
    core_s = extent(hsv[:, 1])
    core_v = extent(hsv[:, 2])
    return Window(
        name=name, avoid=name in AVOID, samples=samples, hsv=hsv, center=center,
        core_h=core_h, core_s=core_s, core_v=core_v,
        h=[core_h[0] - hue_margin, core_h[1] + hue_margin],
        s=[max(0, core_s[0] - sv_margin), min(255, core_s[1] + sv_margin)],
        v=[max(0, core_v[0] - sv_margin), min(255, core_v[1] + sv_margin)],
    )


def hue_frame(a, b):
    """The multiple of 180 that puts b's hue next to a's, for comparing them."""
    return min((-HUE_MAX, 0, HUE_MAX), key=lambda k: abs(b.center + k - a.center))


def intervals_meet(x, y):
    return max(x[0], y[0]) <= min(x[1], y[1])


def windows_overlap(a, b):
    k = hue_frame(a, b)
    return (intervals_meet(a.h, [b.h[0] + k, b.h[1] + k])
            and intervals_meet(a.s, b.s) and intervals_meet(a.v, b.v))


def separate(a, b, hue_margin, sv_margin):
    """
    Cuts two overlapping windows apart between their samples.

    The cut goes on the channel whose SAMPLES are furthest apart, measured in
    margins - a 7-unit hue gap is huge, a 7-unit saturation gap is lighting
    noise - and lands halfway across the gap, so each color keeps an even
    share of the space between them.

    I/O:
        return: the channel cut ("H", "S" or "V"), or None if the samples
                themselves overlap on every channel and no box can split them
    """
    k = hue_frame(a, b)
    b_core_h = [b.core_h[0] + k, b.core_h[1] + k]
    a_below_on_hue = a.center < b.center + k
    candidates = {
        "S": (max(a.core_s[0], b.core_s[0]) - min(a.core_s[1], b.core_s[1]), max(1, sv_margin)),
        "V": (max(a.core_v[0], b.core_v[0]) - min(a.core_v[1], b.core_v[1]), max(1, sv_margin)),
    }
    # Never cut a color from white or black on hue: their hue is noise (grey
    # has no hue), so a white sample at hue 170 says nothing about the next
    # white pixel - only "low saturation" / "low value" separates them.
    if not (a.avoid or b.avoid):
        candidates["H"] = ((b_core_h[0] - a.core_h[1]) if a_below_on_hue
                           else (a.core_h[0] - b_core_h[1]), max(1, hue_margin))
    channel, (gap, scale) = max(candidates.items(), key=lambda item: item[1][0] / item[1][1])
    if gap <= 0:
        return None

    if channel == "H":
        lower, upper = (a, b) if a_below_on_hue else (b, a)
        shift = k if upper is b else -k        # upper's hue, in lower's frame
        cut = math.floor((lower.core_h[1] + upper.core_h[0] + shift) / 2)
        lower.h[1] = min(lower.h[1], cut)
        upper.h[0] = max(upper.h[0], cut + 1 - shift)
    else:
        attr = channel.lower()
        core_a, core_b = getattr(a, "core_" + attr), getattr(b, "core_" + attr)
        lower, upper = (a, b) if core_a[1] < core_b[0] else (b, a)
        lower_core, upper_core = getattr(lower, "core_" + attr), getattr(upper, "core_" + attr)
        cut = math.floor((lower_core[1] + upper_core[0]) / 2)
        getattr(lower, attr)[1] = min(getattr(lower, attr)[1], cut)
        getattr(upper, attr)[0] = max(getattr(upper, attr)[0], cut + 1)
    return channel


def anchor_for_hue(hue, s, v):
    """
    An RGB triple whose OpenCV hue is exactly `hue`.

    COLOR_SPECS takes the hue from the anchor's RGB, and an HSV->RGB->HSV round
    trip can land a unit off, so this searches until it round-trips exactly.
    Tries the samples' own saturation/value first, so the anchor still looks
    like the color.
    """
    for sat, val in ((s, v), (max(s, 150), max(v, 150)), (255, 255)):
        for nudge in (0, -1, 1, -2, 2):
            pixel = np.uint8([[[(hue + nudge) % HUE_MAX, sat, val]]])
            rgb = tuple(int(c) for c in cv2.cvtColor(pixel, cv2.COLOR_HSV2RGB)[0][0])
            if rgb_to_hue(rgb) == hue:
                return rgb
    raise ValueError(f"no RGB round-trips to hue {hue}")


def to_spec(window):
    """
    The window as a COLOR_SPECS row: (anchor RGB, hue_tol, S band, V band).

    A spec's hue window is symmetric about its anchor, so an even-width window
    gives up one unit - taken from whichever end has more room past the samples.
    """
    low, high = window.h
    tol = (high - low) // 2
    if (high - window.core_h[1]) >= (window.core_h[0] - low):
        center = low + tol
    else:
        center = high - tol
    s = int(np.median(window.hsv[:, 1]))
    v = int(np.median(window.hsv[:, 2]))
    anchor = anchor_for_hue(int(center) % HUE_MAX, s, v)
    return anchor, tol, tuple(window.s), tuple(window.v)


# A colour is never thinned below this many values by conflict clean-up; past
# that the two colours are split by force instead.
MIN_KEPT_SAMPLES = 2
MAX_CLEANUP_ROUNDS = 20


def force_separate(a, b):
    """
    Splits two colours whose samples overlap on EVERY channel - which
    separate() refuses - by cutting halfway between their MEDIANS on the
    channel where those medians are furthest apart (in margins, as separate()
    measures). Each colour gives up whatever of its values lie past the cut;
    in return no pixel can match both.

    I/O:
        return: the channel cut ("H", "S" or "V")
    """
    k = hue_frame(a, b)
    a_h = np.median(hue_offsets(a.hsv[:, 0], a.center)) + a.center
    b_h = np.median(hue_offsets(b.hsv[:, 0], b.center)) + b.center + k
    medians = {
        "S": (np.median(a.hsv[:, 1]), np.median(b.hsv[:, 1]), 15.0),
        "V": (np.median(a.hsv[:, 2]), np.median(b.hsv[:, 2]), 15.0),
    }
    if not (a.avoid or b.avoid):
        medians["H"] = (a_h, b_h, 3.0)
    channel = max(medians, key=lambda c: abs(medians[c][0] - medians[c][1]) / medians[c][2])
    a_mid, b_mid, _ = medians[channel]
    cut = math.floor((a_mid + b_mid) / 2)

    def clip(window, attr, low=None, high=None):
        for name in (attr, "core_" + attr):
            bounds = getattr(window, name)
            if low is not None:
                bounds[0] = max(bounds[0], low)
            if high is not None:
                bounds[1] = min(bounds[1], high)
            bounds[1] = max(bounds[1], bounds[0])     # never an empty interval

    attr = channel.lower()
    a_lower = a_mid <= b_mid
    if channel == "H":
        if a_lower:
            clip(a, "h", high=cut)
            clip(b, "h", low=cut + 1 - k)
        else:
            clip(b, "h", high=cut - k)
            clip(a, "h", low=cut + 1)
    else:
        lower, upper = (a, b) if a_lower else (b, a)
        clip(lower, attr, high=cut)
        clip(upper, attr, low=cut + 1)
    return channel


def strayness(window, rgb):
    """How far a sample sits from the rest of its own colour, in MADs (hue or saturation)."""
    h, sat, _ = rgb_to_hsv([rgb])[0]
    offsets = hue_offsets(window.hsv[:, 0], window.center)
    mine = hue_offsets([h], window.center)[0]
    sats = window.hsv[:, 1].astype(float)
    h_mad = max(np.median(np.abs(offsets - np.median(offsets))), OUTLIER_FLOOR["H"])
    s_mad = max(np.median(np.abs(sats - np.median(sats))), OUTLIER_FLOOR["S"])
    return max(abs(mine - np.median(offsets)) / h_mad, abs(sat - np.median(sats)) / s_mad)


def solve(samples_by_name, hue_margin=3, sv_margin=15):
    """
    Solves every color's spec together - and ALWAYS comes back with a spec for
    every color that has samples.

    When two colours' samples overlap on every channel, the values that sit
    among the other colour's (a red click pasted as pink, glare pasted as a
    colour) are dropped and the whole solve starts again. If that cannot
    settle it - the colours genuinely blur into each other - they are split
    by force between their medians (force_separate).

    I/O:
        samples_by_name (dict): {"RED": [(R, G, B), ...], ...}
        returns: (specs, cuts, removed, forced, windows)
            specs: {"RED": (anchor, hue_tol, sat, val), ...}
            cuts: [(a, b, channel), ...] - overlaps cut between the samples
            removed: [(name, rgb, looked_like), ...] - values dropped to settle
                     a conflict
            forced: [(a, b, channel), ...] - pairs split between their medians
            windows: {name: Window}
    """
    samples_by_name = {name: list(samples) for name, samples in samples_by_name.items()}
    removed = []
    for _ in range(MAX_CLEANUP_ROUNDS):
        windows = [build_window(name, samples, hue_margin, sv_margin)
                   for name, samples in samples_by_name.items() if samples]
        cuts, forced = [], []
        restart = False
        # Cuts only shrink windows, so repeat until a full pass changes nothing.
        changed = True
        while changed and not restart:
            changed = False
            for i, a in enumerate(windows):
                for b in windows[i + 1:]:
                    if a.avoid and b.avoid:
                        continue              # white vs black is nobody's business
                    if not windows_overlap(a, b):
                        continue
                    channel = separate(a, b, hue_margin, sv_margin)
                    if channel is not None:
                        cuts.append((a.name, b.name, channel))
                        changed = True
                        continue
                    # Conflict. Drop the values that sit among the other
                    # colour's - only from a real colour, never from WHITE or
                    # BLACK, and never below MIN_KEPT_SAMPLES - and start over.
                    culprits = [(w, other, rgb) for w, other in ((a, b), (b, a)) if not w.avoid
                                for rgb in w.samples if inside_core(other, rgb)]
                    by_name = {}
                    for w, other, rgb in culprits:
                        by_name.setdefault(w.name, []).append((rgb, other.name))
                    dropping = {name: items for name, items in by_name.items()
                                if len(samples_by_name[name]) - len(items) >= MIN_KEPT_SAMPLES}
                    # One side per round - the one with FEWER stray values,
                    # which is the side they really are strays on. A bad pink
                    # value stretches pink over red, so red's own values then
                    # look like pink too; drop the pink one and they don't.
                    if dropping:
                        by_window = {w.name: w for w in (a, b)}
                        fewest = min(len(items) for items in dropping.values())
                        tied = [name for name in dropping if len(dropping[name]) == fewest]
                        # On a tie, the side whose stray sits furthest from
                        # the rest of its OWN colour is the one it strayed into.
                        worst = max(tied, key=lambda name: max(
                            strayness(by_window[name], rgb) for rgb, _ in dropping[name]))
                        dropping = {worst: dropping[worst]}
                        for name, items in dropping.items():
                            for rgb, other in items:
                                samples_by_name[name].remove(rgb)
                                removed.append((name, rgb, other))
                        restart = True
                        break
                    forced.append((a.name, b.name, force_separate(a, b)))
                    changed = True
                if restart:
                    break
        if not restart:
            break
    specs = {w.name: to_spec(w) for w in windows if not w.avoid}
    return specs, cuts, removed, forced, {w.name: w for w in windows}


def inside_core(window, rgb):
    """Whether a sample sits among `window`'s own samples on every channel."""
    h, sat, val = rgb_to_hsv([rgb])[0]
    offset = hue_offsets([h], window.center)[0] + window.center
    hue_ok = window.avoid or window.core_h[0] <= offset <= window.core_h[1]
    return (hue_ok and window.core_s[0] <= sat <= window.core_s[1]
            and window.core_v[0] <= val <= window.core_v[1])


# ============================================================================
# Reporting
# ============================================================================

def verify(specs, samples_by_name):
    """
    Re-checks the result through build_hsv_range itself - the exact bounds the
    robot will use - so what is printed is known to hold, not just intended to.

    I/O:
        returns: (coverage, overlaps)
            coverage: {name: (samples inside, total)}
            overlaps: [(a, b)] that still share pixels, including against the
                      colors NOT being solved (BLUE, WHITE), and any color
                      that still claims a WHITE or BLACK sample
    """
    ranges = {name: build_hsv_range(*spec) for name, spec in specs.items()}
    coverage = {}
    overlaps = []
    for avoid in AVOID:
        pixels = rgb_to_hsv(samples_by_name[avoid]) if samples_by_name.get(avoid) else []
        for name, (lower, upper) in ranges.items():
            caught = sum(in_range(px, lower, upper) for px in pixels)
            if caught:
                overlaps.append((name, f"{caught} of your {avoid} values"))
    for name, samples in samples_by_name.items():
        if name not in ranges:
            continue
        lower, upper = ranges[name]
        inside = sum(in_range(px, lower, upper) for px in rgb_to_hsv(samples))
        coverage[name] = (inside, len(samples))

    others = {color.name: build_hsv_range(*spec) for color, spec in COLOR_SPECS.items()
              if color.name not in ranges and color.name != "ALL_COLORS"}
    names = list(ranges)
    for i, a in enumerate(names):
        for b in names[i + 1:] + list(others):
            other = ranges.get(b) or others[b]
            if all(g <= 0 for g in channel_gaps(ranges[a], other)):
                overlaps.append((a, b if b in ranges else f"{b} (its current spec - not re-solved)"))
    return coverage, overlaps


def format_row(name, spec):
    anchor, tol, saturation, value = spec
    tol = "None" if tol is None else str(tol)
    return (f"    Color.{name + ':':<11} ({str(anchor) + ',':<17} {tol + ',':>5}  "
            f"{str(saturation) + ',':<11} {value}),")


def report(samples_by_name, hue_margin, sv_margin):
    """
    Solves, verifies and prints the whole result, ALWAYS ending with COLOR_SPECS
    rows. Conflicts are settled by solve() itself; what it had to drop or split
    to get there is printed above the rows.

    I/O:
        returns: True if nothing had to be forced and nothing still overlaps
    """
    samples_by_name = dict(samples_by_name)
    dropped_any = False
    for name, samples in samples_by_name.items():
        if name in AVOID or not samples:
            continue
        kept, dropped = drop_outliers(samples)
        if dropped:
            if not dropped_any:
                print("\nDropped as stray values (glare, edges, the wrong block):")
                dropped_any = True
            for rgb in dropped:
                h, sat, val = rgb_to_hsv([rgb])[0]
                print(f"  {name:<7} RGB={rgb}  HSV=({h},{sat},{val})")
        samples_by_name[name] = kept

    specs, cuts, removed, forced, windows = solve(samples_by_name, hue_margin, sv_margin)

    print("\nSamples (H, S, V as measured):")
    for name, samples in samples_by_name.items():
        if not samples:
            print(f"  {name:<7} no samples - skipped")
            continue
        hsv = rgb_to_hsv(samples)
        # Hue the short way round, so a red across the seam reads 176-3, not 0-179.
        center = circular_mean_hue(hsv[:, 0])
        offsets = (hsv[:, 0] - center + HUE_MAX / 2) % HUE_MAX - HUE_MAX / 2
        h_low, h_high = (int(round(center + offsets.min())) % HUE_MAX,
                         int(round(center + offsets.max())) % HUE_MAX)
        hue_span = int(round(offsets.max() - offsets.min()))
        sat_span = int(hsv[:, 1].max() - hsv[:, 1].min())
        hue_text = "any" if name in AVOID else f"{h_low}-{h_high}"
        print(f"  {name:<7} {len(samples):>4} px   H {hue_text}  "
              f"S {hsv[:, 1].min()}-{hsv[:, 1].max()}  V {hsv[:, 2].min()}-{hsv[:, 2].max()}")
        if name not in AVOID and (hue_span > WIDE_HUE or sat_span > WIDE_SATURATION):
            print(f"          ^ very wide for one color - are some of these values "
                  f"from another block, an edge or glare?")

    channel_name = dict(H="hue", S="saturation", V="value")
    if cuts:
        print("\nOverlaps cut apart (between the samples):")
        for a, b, channel in cuts:
            print(f"  {a} / {b}  on {channel_name[channel]}")
    if removed:
        print("\nSkipped because they sat among another color's values (left out of the ranges):")
        for name, rgb, other in removed:
            h, sat, val = rgb_to_hsv([rgb])[0]
            print(f"  {name:<7} RGB={rgb}  HSV=({h},{sat},{val})  looked like {other}")
    if forced:
        print("\nColors that blur into each other, split halfway between their typical values")
        print("(some edge values of each now fall outside - re-click them if detection is patchy):")
        for a, b, channel in forced:
            print(f"  {a} / {b}  on {channel_name[channel]}")

    kept = {name: [rgb for rgb in samples if (name, rgb) not in
                   {(n, r) for n, r, _ in removed}]
            for name, samples in samples_by_name.items()}
    coverage, overlaps = verify(specs, kept)
    print("\nCheck against the real masks:")
    for name, (inside, total) in coverage.items():
        note = "" if inside == total else "  (the rest fall past a cut)"
        print(f"  {name:<7} {inside}/{total} samples inside{note}")
    if overlaps:
        for a, b in overlaps:
            print(f"  WARNING: {a} and {b} can still match the same pixel")
    else:
        print("  no two colors can match the same pixel")

    print("\nPaste into COLOR_SPECS in utils/image_color_utils.py:")
    for name, spec in specs.items():
        print(format_row(name, spec))
    return not forced and not overlaps


# ============================================================================
# Input
# ============================================================================

def read_pasted(name):
    """
    Asks for one color's values and reads them up to an empty line - which is
    the Enter pressed after the paste.
    """
    if name in AVOID:
        print(f"\n{name} ({AVOID[name]} - never to be read as a color): paste the "
              f"values, then press Enter on an empty line (Enter now skips {name})")
    else:
        print(f"\n{name}: paste the values, then press Enter on an empty line "
              f"(Enter now skips {name})")
    samples = []
    while True:
        try:
            line = input(f"{name}> ")
        except EOFError:
            break
        if not line.strip():
            break
        found = parse_samples(line)
        if not found:
            print(f"  no color in that line, ignored: {line.strip()}")
        samples.extend(found)
    if samples:
        hsv = rgb_to_hsv(samples)
        print(f"  got {len(samples)} value(s) for {name}  "
              f"(S {hsv[:, 1].min()}-{hsv[:, 1].max()}, V {hsv[:, 2].min()}-{hsv[:, 2].max()})")
    elif name in AVOID:
        print(f"  {name} skipped - colors will not be checked against it")
    else:
        print(f"  {name} skipped - its current spec stays as it is")
    return samples


def load_samples(path):
    with open(path) as f:
        raw = json.load(f)
    return {name.upper(): [tuple(rgb) for rgb in samples] for name, samples in raw.items()}


def save_samples(path, samples_by_name):
    with open(path, "w") as f:
        json.dump({name: [list(rgb) for rgb in samples]
                   for name, samples in samples_by_name.items()}, f)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--colors", default=",".join(DEFAULT_COLORS),
                        help="colors to solve, in order (default: RED,GREEN,PINK,ORANGE)")
    parser.add_argument("--samples", default=None,
                        help="re-solve values saved earlier with --save")
    parser.add_argument("--save", default=None,
                        help="also save the pasted samples to this JSON file")
    parser.add_argument("--hue-margin", type=int, default=3,
                        help="hue units added past the samples (default 3)")
    parser.add_argument("--sv-margin", type=int, default=15,
                        help="S/V units added past the samples (default 15)")
    args = parser.parse_args()

    names = [name.strip().upper() for name in args.colors.split(",") if name.strip()]
    for name in names:
        if name not in Color.__members__:
            sys.exit(f"Unknown color {name} - Color has {', '.join(Color.__members__)}")

    names += [name for name in AVOID if name not in names]
    if args.samples:
        loaded = load_samples(args.samples)
        samples_by_name = {name: loaded.get(name, []) for name in names}
    else:
        samples_by_name = {name: read_pasted(name) for name in names}
        if args.save:
            save_samples(args.save, samples_by_name)

    if not any(samples_by_name.values()):
        sys.exit("No samples given.")
    report(samples_by_name, args.hue_margin, args.sv_margin)


if __name__ == "__main__":
    main()
