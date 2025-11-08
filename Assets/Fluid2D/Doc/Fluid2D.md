## Record
- overall 2D process 
    - Pressure Apply
    ```
    start();        // initialize var
    update()
        SimStep(deltaTime)
            UPDATE velocity by GRAVITY;
            DENSITY PreCalculate;
            PRESSURE Force and Acc;
            UPDATE velocity by PRESSURE;
            UPDATE position by velocity;
            Collisions();
        END SimStep();
        DRAW Particles;
    END update();
    ```
## TODO
- Damping: for Edge of box
- Viscosity
- Spatial Hash Apply.