using UnityEngine;
using System.Collections;

public class PBD_test: MonoBehaviour 
{
	float 		t = 0.0333f;		// delta T
	float		damping = 0.99f;	// damp force
	int[] 		E;					// edge index
	float[] 	L;					// edge Length
	Vector3[] 	V;					// vertex velocity

	bool windBlow = false;
	float _speed 			= 3.0f;
	Vector3 gravityConst 	= new Vector3(0.0f, -9.8f, 0.0f);
    Vector3 wind 			= new Vector3(6.0f, -2.0f, -15.0f);
	
	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;

		//Resize the mesh.
		int n = 21;
		Vector3[] X  	= new Vector3[n*n];
		Vector2[] UV 	= new Vector2[n*n];
		int[] T	= new int[(n-1)*(n-1)*6];
		for(int j=0; j<n; j++)
			for(int i=0; i<n; i++)
			{
				X[j*n+i] =new Vector3(5-10.0f*i/(n-1), 0, 5-10.0f*j/(n-1));
				UV[j*n+i]=new Vector3(i/(n-1.0f), j/(n-1.0f));
			}
		int t=0;
		for(int j=0; j<n-1; j++)
			for(int i=0; i<n-1; i++)	
			{
				T[t*6+0]=j*n+i;
				T[t*6+1]=j*n+i+1;
				T[t*6+2]=(j+1)*n+i+1;
				T[t*6+3]=j*n+i;
				T[t*6+4]=(j+1)*n+i+1;
				T[t*6+5]=(j+1)*n+i;
				t++;
			}
		mesh.vertices	= X;
		mesh.triangles	= T;
		mesh.uv 		= UV;
		mesh.RecalculateNormals ();

		//Construct the original edge list
		int[] _E = new int[T.Length*2];
		for (int i=0; i<T.Length; i+=3) 
		{
			_E[i*2+0]=T[i+0];
			_E[i*2+1]=T[i+1];
			_E[i*2+2]=T[i+1];
			_E[i*2+3]=T[i+2];
			_E[i*2+4]=T[i+2];
			_E[i*2+5]=T[i+0];
		}
		//Reorder the original edge list
		for (int i=0; i<_E.Length; i+=2)
			if(_E[i] > _E[i + 1]) 
				Swap(ref _E[i], ref _E[i+1]);
		//Sort the original edge list using quicksort
		Quick_Sort (ref _E, 0, _E.Length/2-1);

		int e_number = 0;
		for (int i=0; i<_E.Length; i+=2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
				e_number++;

		E = new int[e_number * 2];
		for (int i=0, e=0; i<_E.Length; i+=2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
			{
				E[e*2+0]=_E [i + 0];
				E[e*2+1]=_E [i + 1];
				e++;
			}

		L = new float[E.Length/2];
		for (int e=0; e<E.Length/2; e++) 
		{
			int i = E[e*2+0];
			int j = E[e*2+1];
			L[e]=(X[i]-X[j]).magnitude;
		}

		V = new Vector3[X.Length];
		for (int i=0; i<X.Length; i++)
			V[i] = new Vector3 (0, 0, 0);
	}

	void Quick_Sort(ref int[] a, int l, int r)
	{
		int j;
		if(l<r)
		{
			j=Quick_Sort_Partition(ref a, l, r);
			Quick_Sort (ref a, l, j-1);
			Quick_Sort (ref a, j+1, r);
		}
	}

	int  Quick_Sort_Partition(ref int[] a, int l, int r)
	{
		int pivot_0, pivot_1, i, j;
		pivot_0 = a [l * 2 + 0];
		pivot_1 = a [l * 2 + 1];
		i = l;
		j = r + 1;
		while (true) 
		{
			do ++i; while( i<=r && (a[i*2]<pivot_0 || a[i*2]==pivot_0 && a[i*2+1]<=pivot_1));
			do --j; while(  a[j*2]>pivot_0 || a[j*2]==pivot_0 && a[j*2+1]> pivot_1);
			if(i>=j)	break;
			Swap(ref a[i*2], ref a[j*2]);
			Swap(ref a[i*2+1], ref a[j*2+1]);
		}
		Swap (ref a [l * 2 + 0], ref a [j * 2 + 0]);
		Swap (ref a [l * 2 + 1], ref a [j * 2 + 1]);
		return j;
	}

	void Swap(ref int a, ref int b)
	{
		int temp = a;
		a = b;
		b = temp;
	}

	void Strain_Limiting()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] posX = mesh.vertices;

		Vector3[] 	Sum_X = new Vector3[posX.Length];
		float[] 	Sum_N = new float[posX.Length];
		// initial counter array
		for (int i = 0; i<posX.Length; i++)
		{
			Sum_X[i] = Vector3.zero;
			Sum_N[i] = 0;
		}
		
		//Apply PBD.
		for (int e = 0; e<E.Length / 2; e++)
		{
			int vi = E[e*2];
			int vj = E[e*2 + 1];
			Vector3 ij_Dir = (posX[vi] - posX[vj] ).normalized;
			Vector3 ij_Sum = posX[vi] + posX[vj];
			Sum_X[vi] += 0.5f * (ij_Sum + L[e]*ij_Dir);
			Sum_X[vj] += 0.5f * (ij_Sum - L[e]*ij_Dir);
			Sum_N[vi] += 1;
			Sum_N[vj] += 1;
		}
		for (int i = 0; i<posX.Length; i++)
		{
			if(i==0 || i==20)	continue;
			Vector3 tempV = (0.2f * posX[i] + Sum_X[i])/(0.2f + Sum_N[i]);
			// warning: Vilocity update first, Vertex after.
			V[i] += (tempV - posX[i]) / t;
			posX[i] = tempV;
		}
		mesh.vertices = posX;
	}

	void Collision_Handling()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;
		
		//For every vertex, detect collision and apply impulse if needed.
		GameObject sphere = GameObject.Find("Sphere");
		Vector3 ObjCenter = sphere.transform.position;

		float radius = 2.7f;
		for(int i = 0; i<X.Length; i++)
		{
			if(i==0 || i==20)	continue;
			Vector3 dis = transform.TransformPoint(X[i]) - ObjCenter;
			if (dis.magnitude < radius)
			{
				Vector3 tempV = ObjCenter + radius * dis.normalized;
				V[i] = (tempV - X[i]) / t; 
				X[i] = tempV;
			}
		}
		mesh.vertices = X;
	}

	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;

		if(Input.GetKey("f")) windBlow = true;
		if(Input.GetKey("r")) windBlow = false;
		// float timeVary = (Time.time *_speed) % 1 + 1.0f;
		float timeVary = (Time.time *_speed) % 2.0f;
		if (timeVary > 1) timeVary = 2.0f - timeVary;
		timeVary += 1.5f;

		for(int i=0; i<X.Length; i++)
		{
			if(i==0 || i==20)	continue;
			//Initial Setup
			if (windBlow) V[i] += t * wind * timeVary;
			V[i] += t * gravityConst;
			V[i] *= damping;
			X[i] += t * V[i];
		}
		mesh.vertices = X;

		// Jacobi method PBD
		for(int l=0; l<32; l++)
			Strain_Limiting ();

		Collision_Handling ();
		mesh.RecalculateNormals ();
	}
}

