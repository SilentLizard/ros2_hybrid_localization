import omni.usd

stage = omni.usd.get_context().get_stage()

path = (
    "/World/heros_3w/Geometry/base_link/base_scan/"
    "SICK_microScan3/Lidar"
)

prim = stage.GetPrimAtPath(path)

print("valid:", prim.IsValid())
print("type:", prim.GetTypeName())

print("\n--- sensor attributes ---")

for attr in prim.GetAttributes():
    name = attr.GetName()

    if any(
        key in name.lower()
        for key in (
            "tick",
            "scan",
            "range",
            "azimuth",
            "elevation",
            "accumulate",
            "rotation",
            "emitter",
        )
    ):
        try:
            print(f"{name:60s} = {attr.Get()}")
        except Exception as exc:
            print(f"{name:60s} = <error: {exc}>")