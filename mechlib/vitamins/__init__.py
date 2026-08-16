"""Bought-part catalog: addressable ISO / datasheet / caliper vitamins.

    from mechlib.vitamins import get, find, all_addresses

    brg = get("bearing/608-2rs")
    pocket_d = brg.od + 0.25          # named product fit, not a second table
    mesh = brg.envelope()             # display only — not a printed STL
"""
from .spec import (
    Vitamin,
    all_addresses,
    all_vitamins,
    find,
    get,
    parse_address,
    reset_index,
)

__all__ = (
    "Vitamin",
    "all_addresses",
    "all_vitamins",
    "find",
    "get",
    "parse_address",
    "reset_index",
)
