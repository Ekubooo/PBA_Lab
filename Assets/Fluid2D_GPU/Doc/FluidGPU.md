## SPH 2D
- TODO: 
    - how compute shader works? compute pipeline.
    - rendering part.

- Overall process of SPH
    ```
    Start()/Init()
        ComputeHelper.SetBuffer();
    END Start()/Init();

    Update()
        RunSimulationFrame()
            UpdateSetting();        // For external changing;
            For: 0 -> IterationPerFrame
  
                RunSimStep()
                    externalForces.Kernel;
                    RunSpatial();   // Process
                    density.Kernel;
                    pressure.Kernel;
                    viscosity.Kernel;
                    updatePosition.Kernel;
                END RunSimStep();

                // detail for Phy standard (See Q&A for details).
                // not be used yet (using by 3D Foam yet).
				SimulationStepCompleted?.Invoke();
            END ForLoop;
        END RunSimulationFrame();
        HandleInput();
    END Update();
    ```

- spatialHash process
    ```
    RunSpatial()
        UpdateSpatialHash.Kernel;

        spatialHash.Run()
            gpuSort.Run() 
                cs.SetBuffer();
                ClearCounts.Kernel;
                Count.Kernel;

                scan.Run();

                ScatterOutputs.Kernel;
                CopyBack.Kernel;
            END gpuSort.Run()

            spatialOffsetsCalc.Run()
                init?.Kernel;
                offsets.Kernel;
            END spatialOffsetsCalc.Run();
        END spatialHash.Run();

        reorder.Kernel;
        copyback.Kernel;
    END RunSpatial();
      
      ```

