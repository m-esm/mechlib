"""Bought-part address, spec, and lookup.

Addresses are ``family/slug`` (PartCAD-style). The tables in this package
are the source of truth for ISO / datasheet / caliper dimensions. Consumers
rebind product params from ``get(address)`` instead of forking a second table.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple

_ADDR_SEP = "/"


@dataclass(frozen=True)
class Vitamin:
    """One bought part. ``dims`` keys are also readable as attributes."""

    address: str
    family: str
    slug: str
    title: str
    source: str
    dims: Mapping[str, Any]
    notes: str = ""

    def __getattr__(self, name: str) -> Any:
        dims = object.__getattribute__(self, "dims")
        if name in dims:
            return dims[name]
        raise AttributeError(name)

    def envelope(self):
        from .envelope import build
        return build(self)


def parse_address(address: str) -> Tuple[str, str]:
    if not isinstance(address, str) or _ADDR_SEP not in address:
        raise ValueError("vitamin address must be family/slug, got %r" % (address,))
    family, slug = address.split(_ADDR_SEP, 1)
    family, slug = family.strip().lower(), slug.strip().lower()
    if not family or not slug or _ADDR_SEP in family:
        raise ValueError("vitamin address must be family/slug, got %r" % (address,))
    return family, slug


def make_vitamin(
    family: str,
    slug: str,
    title: str,
    source: str,
    dims: Mapping[str, Any],
    notes: str = "",
) -> Vitamin:
    family, slug = family.strip().lower(), slug.strip().lower()
    return Vitamin(
        address="%s/%s" % (family, slug),
        family=family,
        slug=slug,
        title=title,
        source=source,
        dims=dict(dims),
        notes=notes,
    )


class VitaminIndex:
    """Addressable catalog built from family tables."""

    def __init__(self, items: Iterable[Vitamin]):
        self._by_addr: Dict[str, Vitamin] = {}
        for item in items:
            if item.address in self._by_addr:
                raise ValueError("duplicate vitamin address %s" % item.address)
            self._by_addr[item.address] = item

    def get(self, address: str) -> Vitamin:
        family, slug = parse_address(address)
        key = "%s/%s" % (family, slug)
        try:
            return self._by_addr[key]
        except KeyError:
            raise KeyError("unknown vitamin %s" % key) from None

    def find(self, query: str) -> List[Vitamin]:
        needle = (query or "").strip().lower()
        if not needle:
            return list(self.all())
        out = []
        for item in self._by_addr.values():
            blob = " ".join((item.address, item.title, item.source, item.notes)).lower()
            if needle in blob:
                out.append(item)
        return out

    def all(self) -> List[Vitamin]:
        return [self._by_addr[k] for k in sorted(self._by_addr)]

    def addresses(self) -> List[str]:
        return sorted(self._by_addr)

    def families(self) -> List[str]:
        return sorted({item.family for item in self._by_addr.values()})


_INDEX: Optional[VitaminIndex] = None


def _index() -> VitaminIndex:
    global _INDEX
    if _INDEX is None:
        from .tables import all_vitamins
        _INDEX = VitaminIndex(all_vitamins())
    return _INDEX


def get(address: str) -> Vitamin:
    return _index().get(address)


def find(query: str = "") -> List[Vitamin]:
    return _index().find(query)


def all_addresses() -> List[str]:
    return _index().addresses()


def all_vitamins() -> List[Vitamin]:
    return _index().all()


def reset_index() -> None:
    """Drop the cached index (tests)."""
    global _INDEX
    _INDEX = None
