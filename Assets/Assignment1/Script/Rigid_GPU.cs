using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

public class Rigid_GPU : MonoBehaviour 
{
    struct RigidBodyData
    {
        public Vector3 position;
        public Vector3 velocity;
        public Vector3 angularVelocity;
        public Quaternion rotation;
    }

    public ComputeShader computeShader;
    public Transform targetTransform;

    ComputeBuffer buffer;
    RigidBodyData[] rbDataArray;

    public float dt = 0.015f;
    public float linearDecay = 0.999f;
    public float angularDecay = 0.98f;
    public Vector3 gravity = new Vector3(0, -9.8f, 0);

    void Start()
    {
        rbDataArray = new RigidBodyData[1];
        rbDataArray[0].position = targetTransform.position;
        rbDataArray[0].velocity = new Vector3(2, 3, 0);
        rbDataArray[0].angularVelocity = new Vector3(0, 0, 2);
        rbDataArray[0].rotation = targetTransform.rotation;

        buffer = new ComputeBuffer(1, sizeof(float) * (3 + 3 + 3 + 4));
        buffer.SetData(rbDataArray);
    }

    void Update()
    {
        computeShader.SetFloat("dt", dt);
        computeShader.SetFloat("linear_decay", linearDecay);
        computeShader.SetFloat("angular_decay", angularDecay);
        computeShader.SetVector("gravity", gravity);
        computeShader.SetBuffer(0, "rigidBodies", buffer);

        computeShader.Dispatch(0, 1, 1, 1);

        buffer.GetData(rbDataArray);
        targetTransform.position = rbDataArray[0].position;
        targetTransform.rotation = rbDataArray[0].rotation;
    }

    void OnDestroy()
    {
        if (buffer != null) buffer.Release();
    }
}
