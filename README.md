# MCmesher
by Kyle J Burgess (github.com/kyy13)


MCmesher is a lightweight marching cubes mesh generator for c++, c#, and Unity.

Features

1. Generate indexed meshes or submeshes with either vertex or face normals
2. Trace rays through a scalar field and detect collision with the mesh


![disp](https://user-images.githubusercontent.com/58697284/154577110-bcabfc84-7365-446f-804d-63e563f7a53b.png)


MCmesher uses lookup tables modified from the transvoxel algorithm [1].

[1] Lengyel, Eric. “Voxel-Based Terrain for Real-Time Virtual Simulations”. PhD diss., University of California at Davis, 2010.
