# vitruvio

## Description

vitruvio contains a method for optimizing quadruped leg designs for a given
trajectory consisting of center of mass position and orientation as well 
as end effector positions and forces over time.

The trajectory data is loaded from a .mat file and the end effector forces
as well as position relative to the hip attachment for that leg are
obtained as an average over a set of steps in the motion. The legs are 
then optimized for that cyclic motion data based on a user-defined cost 
function by means of a genetic algorithm.