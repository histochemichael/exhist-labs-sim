"""Stable illumination for a small rig viewed inside a room-scale scene."""
def apply(model):
    # The benchmark's two nearby lamps produce shadow-map acne far across the
    # room. Disable their shadow maps, not geometry or collision contacts.
    model.light_castshadow[:]=False
    model.vis.headlight.ambient[:]=.3
    model.vis.headlight.diffuse[:]=.35
