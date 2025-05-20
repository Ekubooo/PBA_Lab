using UnityEngine;
using System.Collections;

public class wave : MonoBehaviour 
{
	int size 		= 100;
	float rate 		= 0.005f;
	float gamma		= 0.004f;
	float damping 	= 0.996f;	// Voscosity 
	float[,] 	old_h;
	float[,]	low_h;
	float[,]	vh;
	float[,]	b;

	bool [,]	cg_mask;
	float[,]	cg_p;
	float[,]	cg_r;
	float[,]	cg_Ap;
	bool 	tag = true;

	Vector3 	cube_v = Vector3.zero;
	Vector3 	cube_w = Vector3.zero;


	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.Clear ();

		Vector3[] X = new Vector3[size*size];

		for (int i=0; i<size; i++)
			for (int j=0; j<size; j++) 
			{
				X[i*size+j].x = i*0.1f-size*0.05f;
				X[i*size+j].y = 0;
				X[i*size+j].z = j*0.1f-size*0.05f;
			}

		int[] T = new int[(size - 1) * (size - 1) * 6];
		int index = 0;
		for (int i=0; i<size-1; i++)
			for (int j=0; j<size-1; j++)
			{
				T[index*6+0] = (i+0)*size + (j+0);
				T[index*6+1] = (i+0)*size + (j+1);
				T[index*6+2] = (i+1)*size + (j+1);
				T[index*6+3] = (i+0)*size + (j+0);
				T[index*6+4] = (i+1)*size + (j+1);
				T[index*6+5] = (i+1)*size + (j+0);
				index++;
			}
		mesh.vertices  = X;
		mesh.triangles = T;
		mesh.RecalculateNormals ();

		low_h 	= new float[size,size];
		old_h 	= new float[size,size];
		vh 	  	= new float[size,size];
		b 	  	= new float[size,size];

		cg_mask	= new bool [size,size];
		cg_p 	= new float[size,size];
		cg_r 	= new float[size,size];
		cg_Ap 	= new float[size,size];

		for (int i=0; i<size; i++)
			for (int j=0; j<size; j++) 
			{
				low_h[i,j]=99999;
				old_h[i,j]=0;
				vh[i,j]=0;
			}
	}

	void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
	{
		for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
				if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
				{
					Ax[i,j]=0;
					if(i!=0)		Ax[i,j]-=x[i-1,j]-x[i,j];
					if(i!=size-1)	Ax[i,j]-=x[i+1,j]-x[i,j];
					if(j!=0)		Ax[i,j]-=x[i,j-1]-x[i,j];
					if(j!=size-1)	Ax[i,j]-=x[i,j+1]-x[i,j];
				}
	}

	float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
	{
		float ret=0;
		for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
				if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
				{
					ret+=x[i,j]*y[i,j];
				}
		return ret;
	}

	void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
	{
		//Solve the Laplacian problem by CG.
		A_Times(mask, x, cg_r, li, ui, lj, uj);

		for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
				if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
				{
					cg_p[i,j]=cg_r[i,j]=b[i,j]-cg_r[i,j];
				}

		float rk_norm = Dot(mask, cg_r, cg_r, li, ui, lj, uj);

		for(int k=0; k<128; k++)
		{
			if(rk_norm<1e-10f)	break;
			A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
			float alpha = rk_norm/Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

			for(int i=li; i<=ui; i++)
				for(int j=lj; j<=uj; j++)
					if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
					{
						x[i,j]    += alpha*cg_p[i,j];
						cg_r[i,j] -= alpha*cg_Ap[i,j];
					}

			float _rk_norm = Dot(mask, cg_r, cg_r, li, ui, lj, uj);
			float beta = _rk_norm/rk_norm;
			rk_norm = _rk_norm;

			for(int i=li; i<=ui; i++)
				for(int j=lj; j<=uj; j++)
					if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
						cg_p[i,j] = cg_r[i,j] + beta*cg_p[i,j];
		}
	}

	void coupling1st (string GameObj, ref float[,] old_h, ref float[,] h, ref float [,] new_h)
	{
		// -------------------------------------------------------------
		// Step 2: Block->Water coupling		// ????????
		// TODO: for block 1, calculate low_h.
		GameObject CubeA = GameObject.Find(GameObj);
		Vector3 CubeA_pos = CubeA.transform.position;
		Mesh CubeA_mesh = CubeA.GetComponent<MeshFilter>().mesh;

			// the "AABB" of the Mask, not the real AABB
			// world_pos to water surface_pos(100 * 100); +-3 is for visible effect
			// cube width in world_pos: 1; turn into water_surface width: 10
		int visibleValue 	= 6;
		int leftBoundry 	= (int)((CubeA_pos.x + 5.0f) * 10) - visibleValue;
		int rightBoundry 	= (int)((CubeA_pos.x + 5.0f) * 10) + visibleValue;
		int lowerBoundry 	= (int)((CubeA_pos.z + 5.0f) * 10) - visibleValue;
		int upperBoundry 	= (int)((CubeA_pos.z + 5.0f) * 10) + visibleValue;

		Bounds bounds = CubeA_mesh.bounds;
		Vector3 p = Vector3.zero;
		Vector3 q = Vector3.zero;
		// Traverse the mask
		for (int i = leftBoundry - visibleValue; i <= rightBoundry + visibleValue; i++)
            for (int j = lowerBoundry - visibleValue; j <= upperBoundry + visibleValue; j++)
				if (i >= 0 && j >= 0 && i < size && j < size)
				{
					p = Vector3.zero;
					q = Vector3.zero;
					p = CubeA.transform.InverseTransformPoint
						(new Vector3(i*0.1f - size*0.05f, -10, j*0.1f - size*0.05f));
					q = CubeA.transform.InverseTransformPoint
						(new Vector3(i*0.1f - size*0.05f, -9, j*0.1f - size*0.05f));

					Ray ray = new Ray(p, q - p);
					float dist = 99999;
					bounds.IntersectRay(ray, out dist);
					low_h[i, j] = -10 + dist;	//cube_p.y-0.5f;
				}

		// TODO: then set up b and cg_mask for conjugate gradient.
		for (int i = 0; i < size; i++)
            for (int j = 0; j < size; j++)
            {
				if (low_h[i, j] >= h[i, j])
				{
					b[i, j] = 0;
					vh[i, j] = 0;
					cg_mask[i, j] = false;
				}
				else
				{
					cg_mask[i, j] = true;
					b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
				}
            }
            	
		// TODO: Solve the Poisson equation to obtain vh (virtual height).
		Conjugate_Gradient(cg_mask, b, vh, leftBoundry - 1, rightBoundry + 1, lowerBoundry - 1, upperBoundry + 1);
		// -------------------------------------------------------------
	}

	void Shallow_Wave(float[,] old_h, float[,] h, float [,] new_h)
	{		
		// Step 1:
		// TODO: Compute new_h based on the shallow wave model.
		// using Finite Differencing to simplify the calculation
		// using const value/matrix to replce the pressure item
		// so finally there is only hight in the function
		float sumH = 0.0f;
		for(int i = 0; i < size; i++)
			for(int j = 0; j < size; j++)
				if(i!=0 && j!=0 && i!=size-1 && j!=size-1)
				{
					sumH = 0.0f;
					sumH = h[i+1,j] + h[i-1,j] + h[i,j+1] + h[i,j-1] - 4*h[i,j];
					new_h[i,j] = h[i,j] + (h[i,j] - old_h[i,j]) * damping + sumH * rate;
				}
			
		// -------------------------------------------------------------
		// Step 2: Block->Water coupling		// ????????
		//TODO: for each block, calculate low_h.
		//TODO: then set up b and cg_mask for conjugate gradient.
		//TODO: solve the Poisson equation to obtain vh (virtual height).
		coupling1st("Block", ref old_h, ref h, ref new_h);
		coupling1st("Cube" , ref old_h, ref h, ref new_h);

		// -------------------------------------------------------------
		// TODO: Diminish vh.
		// stable smooth to avoid issue from explict integration
		for(int i = 0; i < size; i++)
			for(int j = 0; j < size; j++)
				if (cg_mask[i, j]) vh[i, j] *= gamma;

		// TODO: Update new_h by vh.
		float sumVH = 0.0f;
		for(int i = 0; i < size; i++)
			for(int j = 0; j < size; j++)
				if(i!=0 && j!=0 && i!=size-1 && j!=size-1)
				{
					sumVH = 0.0f;
					sumVH = vh[i+1,j] + vh[i-1,j] + vh[i,j+1] + vh[i,j-1] - 4*vh[i,j];
					new_h[i,j] += sumVH * rate;
					// new_h[i,j] += vh[i,j] + (vh[i,j] - old_h[i,j]) * damping + sumVH * rate;
				} 
		
		// Step 3
		// TODO: old_h <- h; h <- new_h;
		for(int i = 0; i < size; ++i)
			for(int j = 0; j < size; ++j)
			{
				// h[i,j] = new_h[i,j]; 	// error if let h[i,j] update first
				old_h[i,j] = h[i,j];
				h[i,j] = new_h[i,j];
			}
		
		// Step 4: Water->Block coupling.
		// unfinish
		// More TODO here.
	}
	
	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices;
		float[,] new_h = new float[size, size];
		float[,] h     = new float[size, size];

		// TODO: Load X.y into h.
		for(int i = 0; i < size; ++i)
			for(int j = 0; j < size; ++j)
				h[i,j] = X[i*size + j].y;

		if (Input.GetKeyDown ("r")) 
		{
			// TODO: Add random water.
			int i = Random.Range(1, size-1);
			int j = Random.Range(1, size-1);
			float r = Random.Range(0.02f, 0.05f);
			h[i,j] += 9 * r;

			// volum balance
			for(int a = 0; a < 3; a++)
				for(int b = 0; b < 3; b++)
					h[i-1+a, j-1+b] -= r;
		}
	
		for(int l=0; l<4; l++)
			Shallow_Wave(old_h, h, new_h);
		// Shallow_Wave(old_h, h, new_h);

		//TODO: Store h back into X.y and recalculate normal.
		for(int i = 0; i < size; ++i)
			for(int j = 0; j < size; ++j)
				 X[i*size + j].y = h[i,j];

		mesh.vertices = X;
		mesh.RecalculateNormals();	// Unity Build_In API
	}
}
