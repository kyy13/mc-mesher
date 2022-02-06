# MCmesher
by Kyle J Burgess (github.com/kyy13)


MCmesher is a fast and lightweight marching cubes mesh generator built with chunking/submeshing in mind.

Features

1. Generate indexed meshes or submeshes with either vertex or face normals
2. Trace rays through a scalar field and detect collision with the mesh


![MCmesherImage](https://user-images.githubusercontent.com/58697284/152659021-af65029e-2483-459f-a706-02a2f80e4909.png)


MCmesher uses lookup tables modified from the transvoxel algorithm [1].

[1] Lengyel, Eric. “Voxel-Based Terrain for Real-Time Virtual Simulations”. PhD diss., University of California at Davis, 2010.
