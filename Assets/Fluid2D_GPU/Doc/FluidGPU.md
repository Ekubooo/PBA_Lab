## Overall Process
- Demo process of SPHs
    
    ```
    Start()/Init()
        ComputeHelper.SetBuffer();
    END Start()/Init();

    Update()
        RunSimulationFrame()
            UpdateSetting()
            For:IterationPerFrame
                RunSimStep();
                ?.Invoke();
        HandleInput()
    END Update();
    ```