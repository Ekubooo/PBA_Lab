## Height Field Model
```
begin
Start()
    calculate constant data 
Update()
    user interaction detect
    damping the velocity 
    /// collision impulse //////////////////////
        get quaternion and store as matrix
        for: every point
            collision detect by SDF
            if collided: add point into set
        for: every collided point
            calculate average point
        calculate torque by average point
        calculate collision point velocity by torque
        decompose and recalculate point velocity 
        calculate impulse by point velocity
        update linear and angle velocity by impulse
    /// collision impulse END //////////////////
    update position and orientation
    apply state data to object
end
```



        
        




