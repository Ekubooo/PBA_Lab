## FVM
```
begin
Start()
    load object and create tetrahedra set
    find floor for collection
    calculate constant inverse_Dm
Update()
    input detect
    for: 10-time iterations
    /// FVM ////////////////////////////////
        Deformation Gradient
        Green Strain
        2nd PK Stress
        Elastic Force
        Laplacian Smoothing
        Update position and velocity
        Collision Handle
    /// FVM End/////////////////////////////
    apply vertices set
    recalculate normals
end
```




        
        




