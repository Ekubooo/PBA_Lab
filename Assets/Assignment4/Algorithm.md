## Height Field Model
```
begin
Start()
    pre-process the mesh data
Update()
    load mesh into h[][]
    rain interaction detect
    /// Shallow wave ////////////////////////
    for: n-time iterations  // n define by yourself
        update height by shallow wave equation  
        /// one way coupling ====================
        find object in scene
        find mask by object's position
        using mask to intersection
        set mask, b and virtual hight
        using PCG function to get conjugate gradient
        /// one way coupling End ================
        update height by virtual hight and SW equiation
        update new_h,h,old_h set            // for next loop
    /// Shallow wave End ////////////////////
    data write back into vertices set
    recalculate normals
end
```

## two way coupling
```
begin
Start()
    pre-process the mesh data
Update()
    load mesh into h[][]
    rain interaction detect
    /// Shallow wave ////////////////////////
    for: n-time iterations          
        update height by shallow wave equation  
        /// one way coupling ====================
        find object in scene
        find mask by object's position
        using mask to intersection
        set mask, b and virtual hight
        using PCG function to get conjugate gradient
        /// one way coupling End ================
        update height by virtual hight and SW equiation
        update new_h,h,old_h set
    /// Shallow wave End ////////////////////
    data write back into vertices set
    recalculate normals
end
```



        
        




