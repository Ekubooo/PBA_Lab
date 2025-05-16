# implicit model
```
begin
Start()
    process the square into 20×20×2 mesh
    create edge array       //for spring system
    sort and remove duplicate edges
    create diagonal edge    //diagonal spring
    initialization completed
Update()
    for every X: do
        damping Velocity 
        update X by Velocity
        create X_hat by X
    /// Jacobi Method with Chebyshev //////////////////////////
    for iterate k times: do
        /// Get Gradient ==========================================
        for every X:        
            G[] = M/t^2 * (x1-x0) - f
            gradient calcute by force(gravity)
        for every X: 
            G[] = k(1 - Le/||xi-xj||)(xi-xj)
            gradient calcute by force(Spring)
        /// Gradient End ==========================================
        for every X:                    // Update X by Gradient
            update deltaX by Hessian    // Simplified Hessian
            update deltaX by Chebyshev Acceleration 
    /// Jacobi End/////////////////////////////////////////////
    for every X:   
        update Velocity 
    apply new X position
    Collection handling
end
```

# PBD model


        
        




