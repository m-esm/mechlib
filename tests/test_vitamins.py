"""Vitamin catalog: addresses resolve, ISO numbers stay put, envelopes are volumes."""
from __future__ import annotations

import pytest

from mechlib.vitamins import all_addresses, find, get, parse_address, reset_index
from mechlib.vitamins.tables import all_vitamins


def test_no_duplicate_addresses():
    addrs = [v.address for v in all_vitamins()]
    assert len(addrs) == len(set(addrs))


def test_parse_address_rejects_junk():
    with pytest.raises(ValueError):
        parse_address("608")
    with pytest.raises(ValueError):
        parse_address("/608")
    with pytest.raises(ValueError):
        parse_address("bearing/")


def test_608_and_695_are_iso15():
    b608 = get("bearing/608-2rs")
    assert (b608.id, b608.od, b608.width) == (8.0, 22.0, 7.0)
    b695 = get("bearing/695-2rs")
    assert (b695.id, b695.od, b695.width) == (5.0, 13.0, 4.0)


def test_m3_shcs_and_nut_match_iso():
    screw = get("fastener/iso4762-m3")
    assert screw.head_dk == 5.5
    assert screw.head_k == 3.0
    assert screw.socket_s == 2.5
    assert screw.shank_d == 2.95
    nut = get("nut/din934-m3")
    assert nut.af == 5.5
    assert nut.height == 2.4
    thin = get("nut/din439-m5")
    assert thin.height == 2.7
    assert thin.af == 8.0


def test_n20_and_tcst_caliper_numbers():
    n20 = get("motor/ga12-n20")
    assert n20.shaft_d == 3.0
    assert n20.shaft_flat == 2.33
    assert n20.body_len == 25.3
    assert n20.body_yoff == -0.67
    tcst = get("sensor/tcst1103")
    assert (tcst.body_l, tcst.body_w, tcst.body_h) == (11.9, 6.3, 10.8)


def test_find_bearing_query():
    hits = find("608")
    assert any(v.address == "bearing/608-2rs" for v in hits)
    assert get("bearing/608-2rs") in find("iso15")


def test_unknown_address_raises():
    with pytest.raises(KeyError):
        get("bearing/not-a-real-sku")


def test_every_vitamin_has_provenance_and_dims():
    for item in all_vitamins():
        assert item.source
        assert item.dims
        assert item.family and item.slug
        assert item.address == "%s/%s" % (item.family, item.slug)


def test_envelope_is_volume_for_core_parts():
    for addr in ("bearing/608-2rs", "bearing/695-2rs",
                 "fastener/iso4762-m3", "nut/din934-m3",
                 "washer/iso7089-m3", "cell/18650",
                 "motor/ga12-n20", "sensor/tcst1103"):
        mesh = get(addr).envelope()
        assert mesh.is_volume, addr
        assert mesh.volume > 0.0, addr


def test_address_list_is_sorted_and_stable():
    reset_index()
    a = all_addresses()
    assert a == sorted(a)
    assert "bearing/608-2rs" in a
    assert "fastener/iso4762-m8" in a
