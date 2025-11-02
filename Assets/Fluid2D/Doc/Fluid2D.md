## Record
- overall process 
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
- Damping apply not correct, maybe applying on velosity and update early than simstep, ref to 103
- Position update and draw not align.
- Random dir not implement.