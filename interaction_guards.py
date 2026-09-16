"""Fail-closed boundaries between choreography and unvalidated mechanisms."""

def transfer_blockers(transfer,kind):
    reasons=[]
    if 'baking' in (transfer['source'],transfer['target']):
        reasons.append('Incubator: left-hand handle contact, hinge force and shelf insertion are not integrated.')
    if any(s.startswith('imaging') for s in (transfer['source'],transfer['target'])):
        reasons.append('Scanner: hand-opened door and left-hand cassette seating require contact validation; CAD depth is provisional.')
    if kind=='output_magazine':
        reasons.append('CV5030 output magazine: Nori frame grip and machine removal are not contact-validated.')
    if kind=='scanner_cassette':
        reasons.append('Left-hand scanner carrier grip is a candidate pose, not a validated loaded grasp.')
    if kind=='slide_folder':
        reasons.append('Left-hand folder-edge pinch and loaded folder retention require contact validation.')
    if transfer['target'] in ('routine_a','routine_b'):
        reasons.append('Leica input: drawer-handle contact and carrier seating remain unvalidated.')
    if 'special' in (transfer['source'],transfer['target']):
        from lehisto_loaded_reach import profile
        p=profile()
        reasons.append(p['name']+': lab shoulder height, tilted grasp and moving-rail route differ from the parked-rail loaded tests; collision and rim-clearance validation remain required.')
    return reasons

def hold_time(transfer,kind):
    # Stop before attempting a source machine pickup; otherwise arrive at the
    # destination still holding the rack and stop before simulated insertion.
    if transfer['source']=='baking' or transfer['source'].startswith(('imaging','routine')) or kind in ('output_magazine','scanner_cassette','slide_folder'):
        return transfer['approach']
    return transfer['approach']+2+transfer['travel']
