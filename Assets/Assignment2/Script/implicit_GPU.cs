using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class implicit_GPU : MonoBehaviour
{
	float 		t 			= 0.0333f;		// time step
	float 		mass		= 1;
	float		damping		= 0.99f;
	float 		rho			= 0.995f;		// rho is the estimated spectral radius 
											// of the iterative matrix.
	float 		spring_k 	= 8000;
	int[] 		E;
	float[] 	L;
	Vector3[] 	V;

	Vector3 gravityConst = new Vector3(0.0f,-9.8f,0.0f);
    public ComputeShader _CS;			

	ComputeBuffer _X;
	ComputeBuffer _X_HAT;
	ComputeBuffer _GRADIENT;
	ComputeBuffer _VELOCITY;
	ComputeBuffer _LAST_X;

	ComputeBuffer _EDGE;
	ComputeBuffer _LENGTH;

	int ID_forceGradient;
	int ID_springGradient;
	int ID_updateX;
	int ID_updateV;
	int groupNumX;
	int groupNumE;

    // Start is called before the first frame update
    void Start()
    {
		Mesh mesh = GetComponent<MeshFilter> ().mesh;

		//Resize the mesh.
		int n = 21;
		Vector3[] X  	= new Vector3[n*n];
		Vector2[] UV 	= new Vector2[n*n];
		int[] triangles	= new int[(n-1)*(n-1)*6];
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
			triangles[t*6+0]=j*n+i;
			triangles[t*6+1]=j*n+i+1;
			triangles[t*6+2]=(j+1)*n+i+1;
			triangles[t*6+3]=j*n+i;
			triangles[t*6+4]=(j+1)*n+i+1;
			triangles[t*6+5]=(j+1)*n+i;
			t++;
		}
		mesh.vertices=X;
		mesh.triangles=triangles;
		mesh.uv = UV;
		mesh.RecalculateNormals ();


		//Construct the original E
		int[] _E = new int[triangles.Length*2];
		for (int i=0; i<triangles.Length; i+=3) 
		{
			_E[i*2+0] = triangles[i+0];
			_E[i*2+1] = triangles[i+1];
			_E[i*2+2] = triangles[i+1];
			_E[i*2+3] = triangles[i+2];
			_E[i*2+4] = triangles[i+2];
			_E[i*2+5] = triangles[i+0];
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
			int v0 = E[e*2+0];
			int v1 = E[e*2+1];
			L[e]=(X[v0]-X[v1]).magnitude;
		}

		V = new Vector3[X.Length];
		for (int i=0; i<V.Length; i++)
			V[i] = new Vector3 (0, 0, 0);

		// cs
		int _XLength = X.Length;
		int _ELength = E.Length/2;
		_X 			= new ComputeBuffer(_XLength, 12);	// Vector3
		_X_HAT 		= new ComputeBuffer(_XLength, 12);	// Vector3
		_GRADIENT 	= new ComputeBuffer(_XLength, 12); 	// Vector3
		_VELOCITY 	= new ComputeBuffer(_XLength, 12); 	// Vector3
		_LAST_X 	= new ComputeBuffer(_XLength, 12); 	// Vector3

		_EDGE 		= new ComputeBuffer(E.Length, 4);	// int
		_LENGTH 	= new ComputeBuffer(L.Length, 4);	// float

		ID_forceGradient 	= _CS.FindKernel("forceGradient");
		ID_springGradient 	= _CS.FindKernel("springGradient");
		ID_updateX 			= _CS.FindKernel("updateX");
		ID_updateV 			= _CS.FindKernel("updateV");
		groupNumX 			= Mathf.CeilToInt((float)_XLength / 64.0f);
		groupNumE 			= Mathf.CeilToInt((float)_ELength / 64.0f);
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

	void Collision_Handling()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;
		GameObject sphere = GameObject.Find("Sphere");

		//Handle colllision.
		Vector3 center = sphere.transform.position;
		float radius = 2.7f;
		for(int i = 0; i < X.Length; i++)
		{
			if (i == 0 || i == 20) continue;
			Vector3 d = X[i] - center;
			if (d.magnitude < radius)
			{
				V[i] += (center + radius*d.normalized - X[i])/t;
				X[i] = center + radius*d.normalized;
			}
		}
		mesh.vertices = X;
	}

	void Get_Gradient(Vector3[] X, Vector3[] X_hat, float t, Vector3[] G)
	{
		//Momentum and Gravity.
			// every gradient of points, in G[]
			// (implicit) gradient = M/t^2 * (x1-x0) - ///force(x1)///
			// force(gravity) part
			// can mass be different and store into an array mass[]? maybe
		for (int i = 0; i < G.Length; i++)
			G[i] = (mass/(t*t)) * (X[i]-X_hat[i]) - mass * gravityConst;

		//Spring Force. every edge
			// force(spring) part
		for (int e = 0; e < E.Length/2; e++)
		{
			int i = E[e * 2 + 0];
			int j = E[e * 2 + 1];
			// 1D Spring gradient of Energy (L05P22)
			Vector3 f = spring_k * (1 - L[e]/(X[i]-X[j]).magnitude) * (X[i] - X[j]);

			G[i] += f;
			G[j] -= f;
		}
	}

	void GPU_Gradient(Vector3[] X, Vector3[] X_hat, float t, Vector3[] G, float omega)
	{
		Vector3 outForce = Vector3.zero;
		Vector4 totalForce = gravityConst + outForce;
		Vector4 totalForce4 = new Vector4(totalForce.x,totalForce.y,totalForce.z, 0.0f);
		_X.SetData(X);
		_X_HAT.SetData(X_hat);
		_GRADIENT.SetData(G);

		_EDGE.SetData(E);
		_LENGTH.SetData(L);
		_CS.SetBuffer(ID_forceGradient, "_X", _X);	
		_CS.SetBuffer(ID_forceGradient, "_X_hat", _X_HAT);	
		_CS.SetBuffer(ID_forceGradient, "_GRADIENT", _GRADIENT);

		_CS.SetBuffer(ID_springGradient, "_EDGE", _EDGE);
		_CS.SetBuffer(ID_springGradient, "_LENGTH", _LENGTH);	

		_CS.SetFloat("t", t);
		_CS.SetFloat("mass", mass);
		_CS.SetFloat("spring_k", spring_k);
		_CS.SetFloat("omega", omega);
		_CS.SetVector("forcesC", totalForce4);
		_CS.Dispatch(ID_forceGradient, groupNumX, 1, 1);	

		_CS.SetFloat("edgeCount",E.Length);
		_CS.Dispatch(ID_springGradient, groupNumE, 1, 1);
		_GRADIENT.GetData(G);	//?
	}

 
	void Initial(Mesh mesh)
	{
		Vector3[] X 		= mesh.vertices;
		Vector3[] last_X 	= new Vector3[X.Length];
		Vector3[] X_hat 	= new Vector3[X.Length];
		Vector3[] G 		= new Vector3[X.Length];

		Vector3 outForce = Vector3.zero;
		Vector4 totalForce = gravityConst + outForce;
		Vector4 totalForce4 = new Vector4(totalForce.x,totalForce.y,totalForce.z, 0.0f);

		// 01 Initial Setup.
		for (int i = 0; i < X.Length; i++)
		{
			V[i] *= damping;
			X_hat[i] = X[i] + t*V[i];
			// X[i] = X_hat[i] = X[i] + t*V[i];
			X[i] = X_hat[i];
		}
		// Initial();
		_X.SetData(X);
		_X_HAT.SetData(X_hat);
		_GRADIENT.SetData(G);
		_LAST_X.SetData(last_X);
		_VELOCITY.SetData(V);

		_EDGE.SetData(E);
		_LENGTH.SetData(L);
		_CS.SetBuffer(ID_forceGradient, "_X", _X);	
		_CS.SetBuffer(ID_forceGradient, "_X_hat", _X_HAT);	
		_CS.SetBuffer(ID_forceGradient, "_Gradient", _GRADIENT);

		_CS.SetBuffer(ID_springGradient, "_X", _X);	
		_CS.SetBuffer(ID_springGradient, "_Last_X", _LAST_X);
		_CS.SetBuffer(ID_springGradient, "_Velocity", _VELOCITY);

		_CS.SetBuffer(ID_updateX, "_X", _X);	
		_CS.SetBuffer(ID_updateX, "_EDGE", _EDGE);

		_CS.SetBuffer(ID_updateV, "_X", _X);	
		_CS.SetBuffer(ID_updateV, "_X_hat", _X_HAT);	
		_CS.SetBuffer(ID_updateV, "_LENGTH", _LENGTH);	

		_CS.SetFloat("t", t);
		_CS.SetFloat("mass", mass);
		_CS.SetFloat("spring_k", spring_k);
		_CS.SetFloat("edgeCount",E.Length);

		_CS.SetVector("forcesC", totalForce4);
	} 

	void UpdatePriviousVersion () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X 		= mesh.vertices;
		Vector3[] last_X 	= new Vector3[X.Length];
		Vector3[] X_hat 	= new Vector3[X.Length];
		Vector3[] G 		= new Vector3[X.Length];

		//Initial Setup.
		for (int i = 0; i < X.Length; i++)
		{
			V[i] *= damping;
			X_hat[i] = X[i] + t*V[i];
			// X[i] = X_hat[i] = X[i] + t*V[i];
			X[i] = X_hat[i];
		}
		
		// =============================================
		// The Jacobi Method(not Newton Method) with Chebyshev Acceleration 
		float omega = 1.0f;
		for(int k=0; k<32; k++)
		{
			if (k == 0)			omega = 1.0f;
			else if (k == 1)	omega = 2.0f / (2.0f - rho * rho);
			else			 	omega = 4.0f / (4.0f - rho * rho * omega);

			// (x, x guass, time step, Gradient of every point)
			Get_Gradient(X, X_hat, t, G);
			//Update X by gradient.
			for(int i = 0; i < X.Length; i++)
			{
				if (i == 0 || i == 20) continue;

				// the simple update (lab2.pdf) considering spring as 1D
				// considering the Hessian as a diagonal matrix
				// so, in the Algorithm, deltaX replace by spring_k and stuff.
				Vector3 X_new = 
					omega 		* (X[i] - 1/(mass/(t*t) + 4*spring_k) * G[i]) +
					(1 - omega) * last_X[i];
				last_X[i] = X[i];
				X[i] = X_new;
			} 
		}
		// =============================================
		for(int i = 0; i < X.Length; i++)
			V[i] += (X[i] - X_hat[i]) / t;
		
		//Finishing.
		mesh.vertices = X;
		Collision_Handling ();
		mesh.RecalculateNormals ();
	}

	void Update () 		// GPU version
	{
		// 01 Initial Setup.
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Initial(mesh);
		// =============================================
		// 02 Calculate 
		float omega = 1.0f;
		for(int k=0; k<32; k++)
		{
			if (k == 0)			omega = 1.0f;
			else if (k == 1)	omega = 2.0f / (2.0f - rho * rho);
			else			 	omega = 4.0f / (4.0f - rho * rho * omega);

			// GPU_Gradient(X, X_hat, t, G, omega);
			_CS.SetFloat("omega", omega);
			_CS.Dispatch(ID_forceGradient, groupNumX, 1, 1);

			// pass G to next kernel ?
			_CS.Dispatch(ID_springGradient, groupNumE, 1, 1);

			//Update X by gradient.
			_CS.Dispatch(ID_updateX, groupNumX, 1, 1);
		}
		// =============================================
		_CS.Dispatch(ID_updateV, groupNumX, 1, 1);

		// 03 Pull out data here
		_VELOCITY.GetData(V);
		_X.GetData(mesh.vertices);

		Collision_Handling ();
		mesh.RecalculateNormals ();
	}
}
