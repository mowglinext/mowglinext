# CoverageSession — the behavior tree's live per-session coverage bookkeeping.
#
# Published at 1 Hz on /behavior_tree_node/coverage_session alongside
# HighLevelStatus. It exposes what the resume file (coverage_resume.txt)
# records — which areas this robot has finished or given up on THIS session —
# so a fleet coordinator on another robot can keep its own mower off them.
# Consumed by the GUI fleet coordinator (docs/MULTI_ROBOT.md); nothing on the
# robot itself reads it.
#
# "Completed" is the BT swath-completion model (every drivable sub-path of the
# area was driven), NOT the mow_progress grid. Both sets are cleared by
# EndSession and by ~/clear_coverage_resume ("Start fresh").

# True while a mowing session is live (current_command == COMMAND_START).
bool session_active

# Area index being mowed right now, -1 when none is dispatched.
int16 current_area

# Areas finished this session (index into map_server's area list).
uint32[] completed_areas

# Areas that exhausted their no-progress budget this session (skipped).
uint32[] attempted_areas

# Areas this robot is currently told to leave to another fleet member
# (~/set_fleet_assignment). Empty when the fleet coordinator is off.
uint32[] excluded_areas
