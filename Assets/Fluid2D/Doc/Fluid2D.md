## Record
- overall 2D process 
    - Pressure Apply
    ```
    start();        // initialize var
    update()
        SimStep(deltaTime)
            UPDATE velocity by GRAVITY;
            INIT predictPos;
            DENSITY PreCalculate by predictPos;
            PRESSURE Force and Acc by predictPos;
            UPDATE velocity by PRESSURE;
            UPDATE position by velocity;
            Collisions Handle;
        END SimStep();
        DRAW Particles;
    END update();
    ```
## TODO
- Damping: for Edge of box
- Viscosity
- Spatial Hash Apply.