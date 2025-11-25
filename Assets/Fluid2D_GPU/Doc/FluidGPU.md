## Overall Process
- Demo process of SPH
    
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
  
                    RunSpatial()
                        spatialHash.Kernel;
                        spatialHash.Run()
                            gpuSort.Run();
                            spatialOffsetsCalc.Run();
                        reorder.Kernel;
                        copyback.Kernel;
                    END RunSpatial();
  
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