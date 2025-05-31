using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

public class Rigid_GPU : MonoBehaviour 
{
	bool launched 		= false;
	bool windBlow		= false;
	float dt 			= 0.015f;
	float mass;			
	float restitution 	= 0.5f;                 // for collision
    float friction 		= 0.2f;

    Vector3[] vertices;
	Vector3 x;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity

	Matrix4x4 I_ref;								// reference inertia
	Mesh mesh;
	Quaternion q;

    Vector3 gravity 	= new Vector3(0.0f, -9.8f, 0.0f);
    Vector3 wind 		= new Vector3(5.0f, 0.0f, -3.0f);
    public ComputeShader computeShader;				// Computer Shader setting

	[Range(0.5f, 0.999f)] 
	public float linear_decay	= 0.999f;			// for velocity decay
	[Range(0.5f, 0.98f)] 
	public float angular_decay	= 0.98f;	

	public struct PointData
	{
		public Vector3 pPos;
    	public int isCollision;
	};
	public struct GlobalData
	{
		public Vector3 avgPoint;
		public int cCounter;
	};
    ComputeBuffer DetectBuffer;
    ComputeBuffer globalDBuffer;

    int kernelID;
	int groupNum;

    void Start () 
	{
        mesh = GetComponent<MeshFilter>().mesh;
        vertices = mesh.vertices;

        float m = 1;
		mass = 0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag = m * vertices[i].sqrMagnitude;
			I_ref[0, 0] += diag;
			I_ref[1, 1] += diag;
			I_ref[2, 2] += diag;
			I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
			I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
			I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
			I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
			I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
			I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
			I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
			I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
			I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
		}
		I_ref [3, 3] = 1;

		DetectBuffer 	= new ComputeBuffer(vertices.Length, 12);
		globalDBuffer 	= new ComputeBuffer(1, 16);

		//PointData[] PointDatas	= new PointData[vertices.Length];
		
		/* for(int i = 0; i < vertices.Length; i++)
		{
			PointDatas[i] = new PointData();
			PointDatas[i].pPos = vertices[i];
			PointDatas[i].isCollision = 0;
		} 
		// DetectBuffer.SetData(PointDatas);
		GlobalData[] theOnly = new GlobalData[1];
		theOnly[0].cCounter = 0;
		theOnly[0].avgPoint = Vector3.zero;
		DetectBuffer.SetData(vertices);
		globalDBuffer.SetData(theOnly);
		*/
		kernelID = computeShader.FindKernel("CollisionDetect");
		groupNum = Mathf.CeilToInt((float)vertices.Length / 64.0f);
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

    private Matrix4x4 Matrix_Subtract(Matrix4x4 a, Matrix4x4 b)
    {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)            
                a[i, j] -= b[i, j];

        return a;
    }

    private Matrix4x4 Matrix_Mulitiply(Matrix4x4 a, float b)
    {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                a[i, j] *= b;
        return a;
    }

    private Quaternion Quaternion_Add(Quaternion a, Quaternion b)
    {
        a.x += b.x;
        a.y += b.y;
        a.z += b.z;
        a.w += b.w;
        return a;
    }

    // In this function, update v and w by the impulse due to the collision with
    //a plane <P, N>
    void Collision_Impulse_GPU(string GamePanel)
	{
		GameObject GoPanel = GameObject.Find(GamePanel);
		Vector3 Panel_pos = GoPanel.transform.position;
	  	Vector3 Panel_normal = GoPanel.transform.up;
		Matrix4x4 q_matrix = Matrix4x4.Rotate(q);
		Vector3[] vertPos = new Vector3[vertices.Length];

		for(int i = 0; i< vertices.Length; i++)
        	vertPos[i] = transform.TransformPoint(vertices[i]); 

		// setting CShader data ///////////////////////////////
		GlobalData[] theOnly = new GlobalData[1];
		theOnly[0].cCounter = 0;
		theOnly[0].avgPoint = Vector3.zero;
		DetectBuffer.SetData(vertPos);
		globalDBuffer.SetData(theOnly);
		
		computeShader.SetInt("vertCount", vertices.Length);			
		computeShader.SetVector("PanelPos", Panel_pos);			
		computeShader.SetVector("PanelNormal", Panel_normal);	
		computeShader.SetVector("objVelocity", v);		
		computeShader.SetVector("objW", w);		
		computeShader.SetVector("objPos", transform.position);		
		computeShader.SetMatrix("worldTrans", transform.localToWorldMatrix);
		computeShader.SetMatrix("qMatrix", q_matrix);

		computeShader.SetBuffer(kernelID, "cPoint", DetectBuffer);	
		computeShader.SetBuffer(kernelID, "gData", globalDBuffer);	
		// computeShader.Dispatch(kernelID, 1, 1, 1);	
		computeShader.Dispatch(kernelID, groupNum, 1, 1);	
		// end setting ////////////////////////////////////////

		GlobalData[] outputG 	= new GlobalData[1];
		globalDBuffer.GetData(outputG);
		Vector3 avgPoint 		= outputG[0].avgPoint;
		int cCounter 			= outputG[0].cCounter;

		if (cCounter == 0) return;

		avgPoint /= cCounter;
		Vector3 R_length 			= q_matrix.MultiplyVector(avgPoint);
		Vector3 CpVelocity 			= v + Vector3.Cross(w, R_length);

		Vector3 CpVelocity_N 		= Panel_normal * Vector3.Dot(Panel_normal, CpVelocity);

		Vector3 CpVelocity_Tan 		= CpVelocity - CpVelocity_N;
		Vector3 CpVelocity_N_New 	= -restitution * CpVelocity_N;

        float alpha = Mathf.Max(1.0f - friction * (1.0f + restitution) * CpVelocity_N.magnitude / CpVelocity_Tan.magnitude, 0.0f);
		Vector3 CpVelocity_Tan_New 	= alpha * CpVelocity_Tan;
		Vector3 CpVelocity_New 		= CpVelocity_N_New + CpVelocity_Tan_New;

        Matrix4x4 RriAcc 			= Get_Cross_Matrix(R_length);

        Matrix4x4 I_Inverse 		= Matrix4x4.Inverse(q_matrix * I_ref * Matrix4x4.Transpose(q_matrix));
        Matrix4x4 IofMass 			= Matrix_Mulitiply(Matrix4x4.identity, 1.0f / mass);
		Matrix4x4 K 				= Matrix_Subtract(IofMass, RriAcc * I_Inverse * RriAcc);
        Vector3 J 					= K.inverse.MultiplyVector(CpVelocity_New - CpVelocity);

        v += 1.0f / mass * J;
        w += I_Inverse.MultiplyVector(Vector3.Cross(R_length, J));
	}
	
	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched = false;
			windBlow = false;
			v = Vector3.zero;
		}
		if(Input.GetKey("f"))
		{
			v = new Vector3 (5, 2, 0);
			launched = true;
		}
		 
		if(Input.GetKey("b"))
		{
			windBlow = true;
			launched = true;
		}
		if(Input.GetKey("p"))
		{
			// wind blow, get some mouse event!
			windBlow = false;
		} 

		if (launched)
		{
			// Part I: Update velocities
			if(windBlow) 
				v += dt * wind; 
			v += dt * gravity;
			v *= linear_decay;
			w *= angular_decay;
			if (Vector3.Magnitude(v) <= 0.05f) 	
				launched = false;

			Collision_Impulse_GPU("ground");
			Collision_Impulse_GPU("backwall");

			Vector3 x0 = transform.position;
			Quaternion q0 = transform.rotation;
			x = x0 + dt * v;
			Vector3 dw = 0.5f * dt * w;
			Quaternion qw = new Quaternion(dw.x, dw.y, dw.z, 0.0f);
			q = Quaternion_Add(q0, qw * q0);

			transform.position = x;
			transform.rotation = q;
		}
	}

	void OnDestroy() 
	{
        DetectBuffer.Release();
        DetectBuffer.Dispose();
		
		globalDBuffer.Release();
        globalDBuffer.Dispose();
    }
}