using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
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
	float blend = 0.1f;

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

	private Matrix4x4 Matrix_Sub(Matrix4x4 a, Matrix4x4 b)
    {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)            
                a[i, j] -= b[i, j];

        return a;
    }

    private Matrix4x4 Matrix_Mul(Matrix4x4 a, float b)
    {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                a[i, j] *= b;
        return a;
    }

	private Matrix4x4 Matrix_Add(Matrix4x4 a, Matrix4x4 b)
	{
		for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)            
                a[i, j] += b[i, j];

        return a;
	}

	private float Matrix_Trace(Matrix4x4 m)
	{
		float Trace = 0f;
		for (int i = 0; i<4; i++)
			Trace += m[i, i];
		
		return Trace;
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
    			V[i].y+=0.2f;
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

    		//TODO: Green Strain
			Matrix4x4 TempM = Matrix_Sub(F.transpose * F, Matrix4x4.identity);
			Matrix4x4 Green = Matrix_Mul(TempM, 0.5f);

    		//TODO: Second PK Stress
			Matrix4x4 partA = Matrix_Mul(Green, 2.0f * stiffness_1);
			Matrix4x4 partB = Matrix_Mul( Matrix4x4.identity, stiffness_0 * Matrix_Trace(Green));
			Matrix4x4 PK2nd = Matrix_Add(partA, partB);

    		//TODO: Elastic Force	// P = F * S
			Matrix4x4 PDimT = F * PK2nd * inv_Dm[tet].transpose;	
			float detDim = -1.0f / (6.0f * inv_Dm[tet].determinant);
			Matrix4x4 ForceMatrix = Matrix_Mul(PDimT,detDim);

			Force[Tet[tet * 4 + 0]] -= ((Vector3)ForceMatrix.GetColumn(0) 
				+ (Vector3)ForceMatrix.GetColumn(1) + (Vector3)ForceMatrix.GetColumn(2));
			Force[Tet[tet * 4 + 1]] += (Vector3)ForceMatrix.GetColumn(0);
			Force[Tet[tet * 4 + 2]] += (Vector3)ForceMatrix.GetColumn(1);
			Force[Tet[tet * 4 + 3]] += (Vector3)ForceMatrix.GetColumn(2);
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
