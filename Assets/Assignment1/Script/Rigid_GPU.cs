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

	Matrix4x4 I_ref;							// reference inertia
	Mesh mesh;
	Quaternion q;

    Vector3 gravity 	= new Vector3(0.0f, -9.8f, 0.0f);
    Vector3 wind 		= new Vector3(5.0f, 0.0f, -3.0f);
    public ComputeShader computeShader;			

	[Range(0.5f, 0.999f)] 
	/* public */ float linear_decay		= 0.999f;		
	[Range(0.5f, 0.98f)] 
	/* public */ float angular_decay	= 0.98f;	

	public struct PointData
	{
		public Vector3 pointPos;
		public int isCollision;
	};
	public struct GlobalData
	{
		public Vector3 avgPoint;
		public int cCounter;
	};

    ComputeBuffer DetectBuffer;
    ComputeBuffer PointBuffer;
    ComputeBuffer globalDBuffer;

    int kernelID;
	int groupNum;

    void Start () 
	{
        mesh = GetComponent<MeshFilter>().mesh;
        vertices = mesh.vertices;

        float m = 1;
		mass = 0;
		int vLength = vertices.Length;
		for (int i=0; i<vLength; i++) 
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

		int structSize = System.Runtime.InteropServices.Marshal.SizeOf(typeof(PointData));
		DetectBuffer 	= new ComputeBuffer(vLength, 12);
		PointBuffer 	= new ComputeBuffer(vLength, 16);
		globalDBuffer 	= new ComputeBuffer(1, 16); 
		kernelID = computeShader.FindKernel("CollisionDetect");
		groupNum = Mathf.CeilToInt((float)vLength / 64);
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

    void Collision_Impulse_GPU(string GamePanel)
	{
		GameObject GoPanel = GameObject.Find(GamePanel);
		Vector3 Panel_pos = GoPanel.transform.position;
	  	Vector3 Panel_normal = GoPanel.transform.up;

		Vector4 PanelPos = Panel_pos;
		Vector4 PanelNormal = Panel_normal;
		Vector4 objVelocity = v;
		Vector4 objW = w;
		
		Matrix4x4 q_matrix = Matrix4x4.Rotate(transform.rotation);

		// set buffer
		Vector3[] vertPos = new Vector3[vertices.Length];
		PointData[] pData = new PointData[vertices.Length];
		GlobalData[] theOnly = new GlobalData[1];

		theOnly[0].cCounter = 0;
		theOnly[0].avgPoint = Vector3.zero;
		for(int i = 0; i< vertices.Length; i++)
		{
        	vertPos[i] = vertices[i]; 
			pData[i].pointPos = transform.TransformPoint(vertices[i]);
			pData[i].isCollision = 0;
		}
		// setting CShader data ///////////////////////////////
		DetectBuffer.SetData(vertPos);
		PointBuffer.SetData(pData);
		globalDBuffer.SetData(theOnly);
		
		computeShader.SetInt("vertCount", vertices.Length);			
		computeShader.SetVector("PanelPos", PanelPos);			
		computeShader.SetVector("PanelNormal", PanelNormal);	
		computeShader.SetVector("objVelocity", objVelocity);		
		computeShader.SetVector("objW", objW);		
		computeShader.SetMatrix("qMatrix", q_matrix);
		// computeShader.SetVector("objPos", transform.position);		

		computeShader.SetBuffer(kernelID, "cPoint", DetectBuffer);	
		computeShader.SetBuffer(kernelID, "pointSet", PointBuffer);	
		computeShader.SetBuffer(kernelID, "gData", globalDBuffer);	
		// computeShader.Dispatch(kernelID, groupNum, 1, 1);	
		computeShader.Dispatch(kernelID, 8, 8, 1);	
		// end setting ////////////////////////////////////////

		GlobalData[] outputG = new GlobalData[1];
		PointData[]  outputP = new PointData[vertices.Length];
		globalDBuffer.GetData(outputG);
		PointBuffer.GetData(outputP);
		/* 
		Vector3 avgPoint 	 = outputG[0].avgPoint;
		int cCounter 		 = outputG[0].cCounter;
		*/
		Vector3 avgPoint = Vector3.zero;
		int cCounter = 0;
		for(int i=0; i<outputP.Length; ++i)
		{
			if(outputP[i].isCollision != 0)	
			{
				avgPoint += outputP[i].pointPos;
				cCounter++;
			}
		}
		if (cCounter == 0) return;
		avgPoint /= (float)cCounter;

		Vector3 R_length = q_matrix.MultiplyVector(avgPoint);
		Vector3 CpVelocity = v + Vector3.Cross(w, R_length);
		Vector3 CpVelocity_N 
			= Panel_normal * Vector3.Dot(Panel_normal, CpVelocity);

		Vector3 CpVelocity_Tan = CpVelocity - CpVelocity_N;
		Vector3 CpVelocity_N_New = -restitution * CpVelocity_N;
        float alpha = Mathf.Max(1.0f - friction * (1.0f + restitution) 
			* CpVelocity_N.magnitude / CpVelocity_Tan.magnitude, 0.0f);
		Vector3 CpVelocity_Tan_New = alpha * CpVelocity_Tan;
		Vector3 CpVelocity_New = CpVelocity_N_New + CpVelocity_Tan_New;
        Matrix4x4 RriAcc = Get_Cross_Matrix(R_length);
        Matrix4x4 I_Inverse = Matrix4x4.Inverse(q_matrix * I_ref * Matrix4x4.Transpose(q_matrix));
        Matrix4x4 IofMass = Matrix_Mulitiply(Matrix4x4.identity, 1.0f / mass);
		Matrix4x4 K = Matrix_Subtract(IofMass, RriAcc * I_Inverse * RriAcc);
        Vector3 J = K.inverse.MultiplyVector(CpVelocity_New - CpVelocity);

        v += 1.0f / mass * J;
        w += I_Inverse.MultiplyVector(Vector3.Cross(R_length, J));
	}
	
    void Collision_Impulse(string GamePanel)
	{
		GameObject GoPanel = GameObject.Find(GamePanel);
		Vector3 Panel_pos = GoPanel.transform.position;
		Vector3 Panel_normal = GoPanel.transform.up;

		// List<Vector3> CollisionPoints = new List<Vector3>();
		Matrix4x4 q_matrix = Matrix4x4.Rotate(q);

		Vector3 avgPoint = Vector3.zero;
		uint cCounter = 0;

        for (int i = 0; i < vertices.Length; i++)
		{
            Vector3 xi = transform.TransformPoint(vertices[i]); 
			
			float sdf  = Vector3.Dot(xi - Panel_pos, Panel_normal);
			if(sdf < 0.0f)			
			{
                Vector3 Rri = q_matrix.MultiplyVector(vertices[i]);
                Vector3 vi = v + Vector3.Cross(w, Rri);

                float viDotN = Vector3.Dot(vi, Panel_normal);
				if(viDotN < 0.0f)	
				{		
					avgPoint += vertices[i];
					cCounter++;
            	}
			}
        }
		/*
		if (CollisionPoints.Count == 0) return;
		for(int i = 0; i < CollisionPoints.Count; i++)
			avgPoint += CollisionPoints[i];
		avgPoint /= CollisionPoints.Count;
		*/
		if (cCounter == 0) return;
		avgPoint /= cCounter;

        Vector3 R_length = q_matrix.MultiplyVector(avgPoint);
		Vector3 CpVelocity = v + Vector3.Cross(w, R_length);

		// Impluse Method
		// Collision point velocity in normal direction of plane
		// value(dot result) * direction of normal
		Vector3 CpVelocity_N 
			= Panel_normal * Vector3.Dot(Panel_normal, CpVelocity);

		// Collision point velocity in tangent direction of plane
		Vector3 CpVelocity_Tan = CpVelocity - CpVelocity_N;
		Vector3 CpVelocity_N_New = -restitution * CpVelocity_N;
		// math or mathf?
        float alpha = Mathf.Max(1.0f - friction * (1.0f + restitution) 
			* CpVelocity_N.magnitude / CpVelocity_Tan.magnitude, 0.0f);
		Vector3 CpVelocity_Tan_New = alpha * CpVelocity_Tan;
		Vector3 CpVelocity_New = CpVelocity_N_New + CpVelocity_Tan_New;

        Matrix4x4 RriAcc = Get_Cross_Matrix(R_length);
		// I_Inverse = inertia.inverse
        Matrix4x4 I_Inverse = Matrix4x4.Inverse(q_matrix * I_ref * Matrix4x4.Transpose(q_matrix));
        Matrix4x4 IofMass = Matrix_Mulitiply(Matrix4x4.identity, 1.0f / mass);
		Matrix4x4 K = Matrix_Subtract(IofMass, RriAcc * I_Inverse * RriAcc);
        Vector3 J = K.inverse.MultiplyVector(CpVelocity_New - CpVelocity);

		// torque is Rri×j; j is Impulse, Force × dt 
		// angular_v × torque is the addition of v
		// decomposition volecity into v and w
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
			if(windBlow) 
				v += dt * wind; 
			v += dt * gravity;
			v *= linear_decay;
			w *= angular_decay;
			if (Vector3.Magnitude(v) <= 0.05f) 	
				launched = false;

			Collision_Impulse_GPU("ground");
			Collision_Impulse_GPU("backwall");
			// Collision_Impulse("ground");
			// Collision_Impulse("backwall");

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
        DetectBuffer.Dispose();
        PointBuffer.Dispose();
        globalDBuffer.Dispose();
    }
}