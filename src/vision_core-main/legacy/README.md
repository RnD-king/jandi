# Legacy line implementation

`line_feature_extractor_velocity_legacy.cpp` is the preserved implementation
from immediately before the compact P2P `LineGuide` was added. It is not part
of the CMake target. The active implementation remains
`src/line_feature_extractor.cpp`.

The backup exists for direct comparison with the previously validated RL
continuous-velocity line-following behavior; it must not be added to the
library target together with the active source.
