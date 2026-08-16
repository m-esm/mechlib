"""ISO / datasheet / caliper tables. Facts, not generators.

Provenance tags:
  iso15         ISO 15 deep-groove ball bearings
  iso4762       ISO 4762 / DIN 912 socket-head cap screws
  din934        DIN 934 hex nuts
  din439        DIN 439 / ISO 4035 thin hex nuts
  iso7089       ISO 7089 / DIN 125 plain washers
  iso261        ISO 261 metric thread pitch
  nopscad       NopSCADlib vitamins/ball_bearings.scad (same ISO numbers)
  bd_warehouse  gumyr/bd_warehouse fastener + bearing tables
  bolts         BOLTS open library of technical specifications
  caliper       measured on the owned part (see notes)
  datasheet     published envelope, still CALIPER BEFORE PRINTING
"""
from __future__ import annotations

from typing import List

from .spec import Vitamin, make_vitamin

# id, od, width mm. ISO 15 + the NopSCADlib miniature set.
_BEARINGS = (
    # slug, title, id, od, width, extra
    ("608-2rs", "608-2RS", 8.0, 22.0, 7.0, {}),
    ("608-zz", "608ZZ", 8.0, 22.0, 7.0, {}),
    ("6000-2rs", "6000-2RS", 10.0, 26.0, 8.0, {}),
    ("6001-2rs", "6001-2RS", 12.0, 28.0, 8.0, {}),
    ("6002-2rs", "6002-2RS", 15.0, 32.0, 9.0, {}),
    ("6200-2rs", "6200-2RS", 10.0, 30.0, 9.0, {}),
    ("6201-2rs", "6201-2RS", 12.0, 32.0, 10.0, {}),
    ("6202-2rs", "6202-2RS", 15.0, 35.0, 11.0, {}),
    ("624-2rs", "624-2RS", 4.0, 13.0, 5.0, {}),
    ("625-2rs", "625-2RS", 5.0, 16.0, 5.0, {}),
    ("635-2rs", "635-2RS", 5.0, 19.0, 6.0, {}),
    ("607-2rs", "607-2RS", 7.0, 19.0, 6.0, {}),
    ("627-2rs", "627-2RS", 7.0, 22.0, 7.0, {}),
    ("609-2rs", "609-2RS", 9.0, 24.0, 7.0, {}),
    ("629-2rs", "629-2RS", 9.0, 26.0, 8.0, {}),
    ("686-zz", "686ZZ", 6.0, 13.0, 5.0, {}),
    ("696-zz", "696ZZ", 6.0, 16.0, 5.0, {}),
    ("695-2rs", "695-2RS", 5.0, 13.0, 4.0, {}),
    ("6800-2rs", "6800-2RS", 10.0, 19.0, 5.0, {}),
    ("6801-2rs", "6801-2RS", 12.0, 21.0, 5.0, {}),
    ("6804-2rs", "6804-2RS", 20.0, 32.0, 7.0, {}),
    ("6808-2rs", "6808-2RS", 40.0, 52.0, 7.0, {}),
    ("6900-2rs", "6900-2RS", 10.0, 22.0, 6.0, {}),
    ("6901-2rs", "6901-2RS", 12.0, 24.0, 6.0, {}),
    ("6902-2rs", "6902-2RS", 15.0, 28.0, 7.0, {}),
    ("f623-2rs", "F623-2RS flanged", 3.0, 10.0, 4.0, {"flange_od": 11.5, "flange_w": 0.8}),
    ("f625-zz", "F625ZZ flanged", 5.0, 16.0, 5.0, {"flange_od": 18.0, "flange_w": 1.0}),
    ("f693-zz", "F693ZZ flanged", 3.0, 8.0, 3.0, {"flange_od": 9.5, "flange_w": 0.6}),
    ("f695-zz", "F695ZZ flanged", 5.0, 13.0, 4.0, {"flange_od": 15.0, "flange_w": 0.8}),
    ("mr63-zz", "MR63ZZ", 3.0, 6.0, 2.5, {}),
    ("mr83-zz", "MR83ZZ", 3.0, 8.0, 3.0, {}),
    ("mr85-zz", "MR85ZZ", 5.0, 8.0, 2.5, {}),
    ("mr93-zz", "MR93ZZ", 3.0, 9.0, 4.0, {}),
    ("mr95-zz", "MR95ZZ", 5.0, 9.0, 3.0, {}),
    ("smr95-zz", "SMR95ZZ", 5.0, 9.0, 2.5, {}),
)

# ISO 4762 dk max, k max, hex socket s. shank_d is display (slightly under major
# so a correct clearance hole does not dig the metal mesh).
_SHCS = (
    # size, dk, k, s, pitch, shank_d
    ("m2", 2.0, 3.8, 2.0, 1.5, 0.40, 1.95),
    ("m2.5", 2.5, 4.5, 2.5, 2.0, 0.45, 2.45),
    ("m3", 3.0, 5.5, 3.0, 2.5, 0.50, 2.95),
    ("m4", 4.0, 7.0, 4.0, 3.0, 0.70, 3.90),
    ("m5", 5.0, 8.5, 5.0, 4.0, 0.80, 4.90),
    ("m6", 6.0, 10.0, 6.0, 5.0, 1.00, 5.90),
    ("m8", 8.0, 13.0, 8.0, 6.0, 1.25, 7.90),
)

# DIN 934 AF, height, hole (display).
_NUTS = (
    ("m2", 2.0, 4.0, 1.6, 2.10),
    ("m2.5", 2.5, 5.0, 2.0, 2.60),
    ("m3", 3.0, 5.5, 2.4, 3.05),
    ("m4", 4.0, 7.0, 3.2, 4.10),
    ("m5", 5.0, 8.0, 4.0, 5.05),
    ("m6", 6.0, 10.0, 5.0, 6.10),
    ("m8", 8.0, 13.0, 6.5, 8.10),
)

_THIN_NUTS = (
    ("m5", 5.0, 8.0, 2.7, 5.05),  # DIN 439 / ISO 4035 — Klonk rose stack
)

# ISO 7089 d1, d2, h
_WASHERS = (
    ("m3", 3.2, 7.0, 0.5),
    ("m4", 4.3, 9.0, 0.8),
    ("m5", 5.3, 10.0, 1.0),
    ("m6", 6.4, 12.0, 1.6),
    ("m8", 8.4, 16.0, 1.6),
)

# Heat-set insert OD + stock lengths (McMaster 94180A-class), from mechlib fasteners.
_INSERTS = (
    ("m2", 3.2, (3.0, 4.0)),
    ("m2.5", 4.0, (4.0, 5.7)),
    ("m3", 4.6, (4.0, 5.7, 8.0)),
    ("m4", 6.3, (6.0, 8.1)),
    ("m5", 7.1, (9.5,)),
    ("m6", 9.5, (12.7,)),
)


def _bearings() -> List[Vitamin]:
    out = []
    for slug, title, id_, od, width, extra in _BEARINGS:
        dims = {"id": id_, "od": od, "width": width}
        dims.update(extra)
        out.append(make_vitamin(
            "bearing", slug, title,
            "iso15+nopscad+bd_warehouse",
            dims,
            notes="ISO 15 numbers; NopSCADlib ball_bearings.scad / bd_warehouse bearing.py agree.",
        ))
    return out


def _fasteners() -> List[Vitamin]:
    out = []
    for slug, d, dk, k, s, pitch, shank in _SHCS:
        out.append(make_vitamin(
            "fastener", "iso4762-%s" % slug, "ISO 4762 %s SHCS" % slug.upper(),
            "iso4762+bd_warehouse+bolts",
            {
                "d": d, "head_dk": dk, "head_k": k, "socket_s": s,
                "pitch": pitch, "shank_d": shank,
                "socket_h": round(0.5 * k, 2),
            },
        ))
    for slug, d, af, h, hole in _NUTS:
        out.append(make_vitamin(
            "nut", "din934-%s" % slug, "DIN 934 %s hex nut" % slug.upper(),
            "din934+bd_warehouse+bolts",
            {"d": d, "af": af, "height": h, "hole_d": hole},
        ))
    for slug, d, af, h, hole in _THIN_NUTS:
        out.append(make_vitamin(
            "nut", "din439-%s" % slug, "DIN 439 %s thin hex nut" % slug.upper(),
            "din439+iso4035",
            {"d": d, "af": af, "height": h, "hole_d": hole, "thin": True},
        ))
    for slug, d1, d2, h in _WASHERS:
        out.append(make_vitamin(
            "washer", "iso7089-%s" % slug, "ISO 7089 %s washer" % slug.upper(),
            "iso7089+bd_warehouse+bolts",
            {"d": float(slug[1:]) if slug != "m2.5" else 2.5,
             "id": d1, "od": d2, "height": h},
        ))
    for slug, od, lengths in _INSERTS:
        out.append(make_vitamin(
            "insert", "heatset-%s" % slug, "Heat-set insert %s" % slug.upper(),
            "mcmaster-94180A-class",
            {"d": float(slug[1:]) if slug != "m2.5" else 2.5,
             "od": od, "lengths": list(lengths)},
        ))
    return out


def _electromech() -> List[Vitamin]:
    return [
        make_vitamin(
            "motor", "ga12-n20", "GA12-N20 6V micro metal gearmotor",
            "caliper",
            {
                "shaft_d": 3.0,
                "shaft_flat": 2.33,
                "shaft_len": 9.3,
                "gb_len": 8.0,
                "env_x": 10.0,
                "env_y": 12.0,
                "body_len": 25.3,
                "body_yoff": -0.67,
            },
            notes="Measured from cad/dc-motor-GA12-N20.stl (Klonk 2026-07-16). CALIPER the real motor.",
        ),
        make_vitamin(
            "motor", "tt-gearmotor", "TT dual-shaft DC gearmotor",
            "caliper",
            {
                "shaft_d": 5.5,
                "shaft_flat": 3.7,
                "shaft_len": 8.8,
                "body_w": 23.0,
                "body_thick": 25.0,
                "gearbox_len": 38.0,
                "can_d": 20.5,
                "can_len": 24.0,
            },
            notes="Klonk caliper of the yellow TT. Scan flanges were error.",
        ),
        make_vitamin(
            "motor", "28byj48", "28BYJ-48 5V unipolar stepper",
            "datasheet",
            {
                "body_d": 28.0,
                "body_h": 19.0,
                "ear_span": 35.0,
                "shaft_d": 5.0,
                "shaft_len": 10.0,
            },
            notes="Published envelope. ESTIMATE — caliper before printing.",
        ),
        make_vitamin(
            "motor", "nema17", "NEMA 17 stepper",
            "datasheet",
            {
                "face": 42.3,
                "bolt_span": 31.0,
                "pilot_d": 22.0,
                "shaft_d": 5.0,
                "shaft_len": 24.0,
                "body_len": 48.0,
            },
            notes="NEMA ICS 16 envelope. Body length varies by stack.",
        ),
        make_vitamin(
            "servo", "sg90", "SG90 9g micro servo (published envelope)",
            "datasheet",
            {
                "long": 22.8,
                "thin": 12.2,
                "tall": 22.5,
                "flange_l": 32.2,
                "flange_t": 2.5,
                "flange_up": 15.9,
                "shaft_in": 5.9,
                "shaft_d": 4.8,
                "boss_d": 11.8,
                "boss_h": 4.0,
                "ear_hole_d": 2.2,
            },
            notes="Nudge datasheet envelope, not a caliper. CALIPER BEFORE PRINTING.",
        ),
        make_vitamin(
            "servo", "mg90s", "MG90S metal-gear 9g servo (calipered)",
            "caliper",
            {
                "body_l": 22.6,
                "body_w": 12.4,
                "ear_span": 32.6,
                "ear_t": 2.6,
                "ear_hole_d": 2.0,
                "shaft_off": 5.9,
                "shaft_d": 4.8,
                "shaft_l": 3.5,
                "cap_h": 6.2,
                "cap_boss_r": 9.0,
                "cap_boss_l": 6.0,
                "spline_tip": 9.5,
            },
            notes="Intercom 2026-08 caliper of cad/intercom/mg90s.step. Not the SG90 datasheet.",
        ),
        make_vitamin(
            "cell", "18650", "18650 Li-ion cell",
            "datasheet",
            {"d": 18.6, "length": 65.2},
            notes="Typical wrapped cell. ESTIMATE — caliper the owned cells.",
        ),
        make_vitamin(
            "cell", "aa", "AA cell",
            "datasheet",
            {"d": 14.5, "length": 50.5},
        ),
        make_vitamin(
            "cell", "aaa", "AAA cell",
            "datasheet",
            {"d": 10.5, "length": 44.5},
        ),
        make_vitamin(
            "sensor", "tcst1103", "Vishay TCST1103 transmissive optosensor",
            "datasheet+caliper",
            {
                "body_l": 11.9,
                "body_w": 6.3,
                "body_h": 10.8,
                "slot_w": 3.1,
            },
            notes="Klonk opt_body_* from the Vishay drawing, checked on owned parts.",
        ),
        make_vitamin(
            "sensor", "hc-sr04", "HC-SR04 ultrasonic module",
            "datasheet",
            {
                "board_l": 45.0,
                "board_w": 20.0,
                "board_t": 1.6,
                "barrel_d": 16.0,
                "barrel_span": 25.0,
            },
            notes="Published module. Prefer cad/converted/hc_sr04.glb for the real mesh.",
        ),
    ]


def all_vitamins() -> List[Vitamin]:
    return _bearings() + _fasteners() + _electromech()
