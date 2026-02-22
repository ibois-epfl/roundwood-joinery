# RoundwoodJoinery
Roundwood Joinery is a Rhino8 plugin for planning roundwood joinery in scanned trunks

> [Note]
## Goal
When building with roundwood, accurately cutting joints of specific surfaces in this somewhat irregular material can be tricky. This Rhino8 grasshopper plugin aims at facilitating this process. It requires as input: a scan of the tree trunk to be used, the relative positions of the joints, and their target surfaces (the surfaces expected in the joint). Based on that, this plugin optimises the positions of the joint in the scanned trunk to achieve the target surfaces.

## Process
````mermaid
flowchart LR
    A(trunk scan) --> E{joinery optimisation}
    B(joint surfaces) --> D(joinery definition)
    C(joint relative positions) --> D(joinery definition)
    D --> E
    E --> F(optimised joint positions)
```