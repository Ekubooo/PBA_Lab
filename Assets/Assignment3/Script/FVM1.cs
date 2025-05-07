using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM1 : MonoBehaviour
{
	float dt 			= 0.003f;
    float mass 			= 1;
	float stiffness_0	= 20000.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;

	int[] 		Tet;		// 4-side volum/tetrahedra
	int tet_number;			// The number of tetrahedra

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				// The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	SVD svd = new SVD();

	float FloorYPos;
	float blend = 0.2f;

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/Assignment3/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/Assignment3/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/

        // Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();

		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

		// get floor y pos to handle collection
		GameObject floorObj = GameObject.Find("Floor");
		FloorYPos = floorObj.transform.position.y;

		//TODO: Need to allocate and assign inv_Dm
		inv_Dm = new Matrix4x4[tet_number];
		for (int i = 0; i<tet_number; i++)
			inv_Dm[i] = Build_Edge_Matrix(i).inverse;
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 Dm = Matrix4x4.zero;
        //TODO: Need to build edge matrix here.
        Vector4 x10 = X[Tet[tet * 4 + 1]] - X[Tet[tet * 4]];
        Vector4 x20 = X[Tet[tet * 4 + 2]] - X[Tet[tet * 4]];
        Vector4 x30 = X[Tet[tet * 4 + 3]] - X[Tet[tet * 4]];
        Dm.SetColumn (0, x10);
        Dm.SetColumn (1, x20);
        Dm.SetColumn (2, x30);
        Dm.SetColumn (3, new Vector4(0, 0, 0, 1));
        return Dm;
    }

	void Laplacian_Smoothing(float blend)
	{
		for (int i = 0; i < number; i++)
		{
			V_sum[i] = Vector3.zero;
			V_num[i] = 0;
		}

		for(int tet=0; tet<tet_number; tet++)
		{
			Vector3 sum = V[Tet[tet * 4 + 0]] + V[Tet[tet * 4 + 1]] + V[Tet[tet * 4 + 2]] + V[Tet[tet * 4 + 3]];
			V_sum[Tet[tet * 4 + 0]] += sum;
			V_sum[Tet[tet * 4 + 1]] += sum;
			V_sum[Tet[tet * 4 + 2]] += sum;
			V_sum[Tet[tet * 4 + 3]] += sum;
			V_num[Tet[tet * 4 + 0]] += 4;
			V_num[Tet[tet * 4 + 1]] += 4;
			V_num[Tet[tet * 4 + 2]] += 4;
			V_num[Tet[tet * 4 + 3]] += 4;
		}

		for(int i = 0; i < number; i++)
		{
			V[i] = (V[i] + blend * V_sum[i] / V_num[i]) / (1 + blend);
		}
	}

    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y += 0.5f;
    	}

    	for(int i=0 ;i<number; i++)
    	{
    		//TODO: Add gravity to Force.
			Force[i] = new Vector3(0.0f, -9.8f*mass, 0.0f);
    	}

    	for(int tet=0; tet<tet_number; tet++)
    	{
    		//TODO: Deformation Gradient
    		Matrix4x4 F = Build_Edge_Matrix(tet) * inv_Dm[tet];

    		//TODO: Second PK Stress
			// method using SVD 

			/* 
			Matrix4x4 U = Matrix4x4.zero;
			Matrix4x4 mid = Matrix4x4.zero;
			Matrix4x4 V = Matrix4x4.zero;
			svd.svd(F, ref U, ref mid, ref V);
			float lambda0 = mid[0, 0];
			float lambda1 = mid[1, 1];
			float lambda2 = mid[2, 2];

			float Ic = lambda0*lambda0 + lambda1*lambda1 + lambda2*lambda2;

			float dWdIc = 0.25f * stiffness_0 * (Ic - 3f) - 0.5f * stiffness_1;
			float dWdIIc = 0.25f * stiffness_1;
			float dIcdlambda0 = 2f * lambda0;
			float dIcdlambda1 = 2f * lambda1;
			float dIcdlambda2 = 2f * lambda2;
			float dIIcdlambda0 = 4f * lambda0 * lambda0 * lambda0;
			float dIIcdlambda1 = 4f * lambda1 * lambda1 * lambda1;
			float dIIcdlambda2 = 4f * lambda2 * lambda2 * lambda2;
			float dWd0 = dWdIc * dIcdlambda0 + dWdIIc * dIIcdlambda0;
			float dWd1 = dWdIc * dIcdlambda1 + dWdIIc * dIIcdlambda1;
			float dWd2 = dWdIc * dIcdlambda2 + dWdIIc * dIIcdlambda2;

			Matrix4x4 diag = Matrix4x4.zero;
			diag[0, 0] = dWd0;
			diag[1, 1] = dWd1;
			diag[2, 2] = dWd2;
			diag[3, 3] = 1;
			Matrix4x4 PK2nd = U * diag * V.transpose;
			*/

			Matrix4x4 U = Matrix4x4.identity;
            Matrix4x4 S = Matrix4x4.identity;
            Matrix4x4 V = Matrix4x4.identity;
            svd.svd(F, ref U, ref S, ref V);
            //right Cauchy-Green deformation tensor
            float[] lambda = new float[3];
            lambda[0] = S[0, 0];
            lambda[1] = S[1, 1];
            lambda[2] = S[2, 2];
            float Ic = lambda[0] * lambda[0] + lambda[1] * lambda[1] + lambda[2] * lambda[2];
            float dW0 = (0.5f * stiffness_0 * (Ic - 3) + 0.5f * stiffness_1 * (lambda[1] * lambda[1] + lambda[2] * lambda[2] - 2)) * lambda[0];
            float dW1 = (0.5f * stiffness_0 * (Ic - 3) + 0.5f * stiffness_1 * (lambda[0] * lambda[0] + lambda[2] * lambda[2] - 2)) * lambda[1];
            float dW2 = (0.5f * stiffness_0 * (Ic - 3) + 0.5f * stiffness_1 * (lambda[0] * lambda[0] + lambda[1] * lambda[1] - 2)) * lambda[2];

            Matrix4x4 W = Matrix4x4.identity;
            W[0, 0] = dW0;
            W[1, 1] = dW1;
            W[2, 2] = dW1;
            Matrix4x4 PK2nd = U * W * V.transpose;

    		//TODO: Elastic Force	// P = F * S
			Matrix4x4 forces = F * PK2nd * inv_Dm[tet].transpose;	
			float detDim = -1.0f / (6.0f * inv_Dm[tet].determinant);
			for (int i = 0; i < 3; ++i)
            	for (int j = 0; j < 3; ++j)
					forces[i,j] *= detDim;

			Force[Tet[tet * 4 + 0]] -= ((Vector3)forces.GetColumn(0) 
				+ (Vector3)forces.GetColumn(1) + (Vector3)forces.GetColumn(2));
			Force[Tet[tet * 4 + 1]] += (Vector3)forces.GetColumn(0);
			Force[Tet[tet * 4 + 2]] += (Vector3)forces.GetColumn(1);
			Force[Tet[tet * 4 + 3]] += (Vector3)forces.GetColumn(2);
    	}
		
		// Using Laplacian Method
		Laplacian_Smoothing(blend);

    	for(int i=0; i<number; i++)
    	{
    		//TODO: Update X and V here.
			V[i] += dt * Force[i] / mass;
			V[i] *= damp;
			X[i] += dt * V[i];

    		//TODO: (Particle) collision with floor.
			if(X[i].y < FloorYPos)
			{
				V[i].y += (FloorYPos -X[i].y) / dt;
				X[i].y = FloorYPos;
			}
    	}
    }

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices = vertices;
		mesh.RecalculateNormals ();
    }
}
