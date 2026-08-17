# Third-party notices

## Innok HEROS robot description

Source project:

- Repository: https://github.com/innokrobotics/innok_heros_description
- Source package version: 1.0.4
- Copyright holder: Innok Robotics GmbH
- Upstream license: BSD (two-clause terms in the upstream xacro sources)

The following files in this repository include or are derived from material
from the Innok HEROS robot-description package:

- `robot/meshes/*.STL`
- `robot/heros_3w_isaac.urdf`
- `robot/heros_3w.usd`
- `robot/heros_3w_isaac/**`
- configuration values explicitly marked as originating from
  `innok_heros_description`

The upstream source identifies the following copyright:

Copyright (c) 2015, Innok Robotics GmbH, All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

## Project-specific simulation additions

Simulation-specific additions in `hybrid_localization_isaac_sim`, including the
SICK scanner fixture, `base_scan` placement, Isaac Sim articulation
configuration, ROS graph integration, generated worlds and maps, and
localization parameters, are additions made by this project and are not claimed
to be Innok Robotics manufacturer specifications.
