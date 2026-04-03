# RoundwoodJoinery
![ubuntu build test](https://github.com/ibois-epfl/roundwood-joinery/actions/workflows/test_linux_build.yml/badge.svg)
![macos build test](https://github.com/ibois-epfl/roundwood-joinery/actions/workflows/test_mac_build.yml/badge.svg)
![windows build test](https://github.com/ibois-epfl/roundwood-joinery/actions/workflows/test_windows_build.yml/badge.svg)

Roundwood Joinery is a Rhino8 plugin for planning roundwood joinery in scanned trunks

> [!Note]
> The main branch is very rudimentary and contains the first milestone of this project: an end-to-end initial test. It has none of the rhino8 plugin infrastructure, and is only here for communication and idea sharing. When the plugin exists and is decent, this warning will be removed. Until then, cheers !

## Goal
![illustratoin_of_beam](./assets/imgs/2026_04_03_whole_beam_pc_illustration.png)
When building with roundwood, accurately cutting joints of specific surfaces in this somewhat irregular material can be tricky. This future Rhino8 grasshopper plugin aims at facilitating this process. It requires as input: a scan of the tree trunk to be used, the relative positions of the joints, and their target surfaces (the surfaces expected in the joint). Based on that, this plugin optimises the positions of the joint in the scanned trunk to achieve the target surfaces.
![illustratoin_of_joint](./assets/imgs/2026_04_03_joint_correction_illustration.png)

Hereunder the joint face area situation of the illustration above
|id|target|initial (red) |after 1 iteration (green)
|--|--|--|--|
|1|5000|<span style="color: #E74C3C;">3573.6</span>  |<span style="color: #2ECC71;">5021.1</span> 
|2|9000|<span style="color: #E74C3C;">8250.7</span>  |<span style="color: #2ECC71;">9126.0</span> 
|3|5000|<span style="color: #E74C3C;">4711.1</span>  |<span style="color: #2ECC71;">6085.4</span> 


## Process
```mermaid
flowchart LR
    A(trunk scan) --> E{joinery optimisation}
    B(joint surfaces) --> D(joinery definition)
    C(joint relative positions) --> D(joinery definition)
    D --> E
    E --> F(optimised joint positions)
```

## Usage
Checkout the [Usage.md](./USAGE.md) for more info, but right now you should probably not try this at home ;) 