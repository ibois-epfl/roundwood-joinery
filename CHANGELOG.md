# Change log
We keep track here of the main changes for each release, starting from 0.0.1

## 0.1.0

### New
- Creation of `JointGroup` class with 1 translation DOF stored in it.
- The DOF's are expressed in the global coord system and are thus not relative anymore but are all related to one reference frame (the CAD's reference system).
    When applying a transformation to a joint group, this joint group's DOF is also transformed, so it can be used in an iterative loop.
- Creation of mean transformation matrix comnputation in the Utils class
- Optimisation loop created and tested on real data (it actually converges, amazing)

### Changed
- 2d Alpha shape now correctly uses the face plane as 2D space, rather than the xy, xz, or yz plane as before

### Removed