#!/usr/bin/env python3
"""Run in Isaac Script Editor to inspect imported HEROS prim names/types."""
import omni.usd
s = omni.usd.get_context().get_stage()
for p in s.Traverse():
    path = str(p.GetPath())
    if "heros" in path.lower() or p.GetTypeName() == "OmniLidar":
        print(f"{path:90s} {p.GetTypeName()}")
