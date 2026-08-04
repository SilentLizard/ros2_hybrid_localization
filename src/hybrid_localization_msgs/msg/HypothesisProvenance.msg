# Compact direct-parent lineage for one Gaussian hypothesis.
# ID zero is reserved as invalid/unassigned.

uint8 EVENT_UNASSIGNED=0
uint8 EVENT_PARTICLE_CLUSTER=1
uint8 EVENT_MERGE=2
uint8 EVENT_SPLIT=3

uint64 id
uint64[2] parent_ids
uint8 parent_count
uint64 generation
uint8 event
