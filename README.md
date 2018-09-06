# matlab-mesh-optimization

This is a program that calculates the minimum possible surface area of a mesh that
has a simple Delaunay triangulation of X and Y lattice points. The height at each
lattice point is able to be fixed, and the remaining unknown heights will be
optimized such that the mesh has the minimim possible surface area. This problem
is a convex optimization problem, in particular, a second order cone program, which
requires an advanced solver to run. Mosek, a solver, is used, as well as YALMIP, the
interface that interacts with that solver. If you are planning to run the code, you
must change the paths to reflect the locations of those two programs. 