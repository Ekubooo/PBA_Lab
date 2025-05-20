// Upgrade NOTE: replaced '_Object2World' with 'unity_ObjectToWorld'
// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

Shader "DynamicWater"
{
    Properties
    {
        _Color ("Main Color", Color) = (0, 0.15, 0.115, 1)
		_MainTex ("Base (RGB)", 2D) = "white" {}
		_WaveMap ("Wave Map", 2D) = "bump" {}
		_Cubemap ("Environment Cubemap", Cube) = "_Skybox" {}
		_WaveXSpeed ("Wave Speed H", Range(-0.1, 0.1)) = 0.01
		_WaveYSpeed ("Wave Speed V", Range(-0.1, 0.1)) = 0.01		
        _Distortion ("Distortion", Range(0, 600)) = 300

    }
    SubShader
    {
		Tags {"Queue" = "Transparent"}
        Tags {"RenderType" = "Opaque" }
		// Screen capture
		GrabPass { "_RefractionTex" }

        Pass
        {
            Tags {"LightMode" = "ForwardBase"}

            CGPROGRAM
            #include "UnityCG.cginc"
			#include "Lighting.cginc"
            #pragma multi_compile_fwdbase
            #pragma vertex vert
            #pragma fragment frag

            fixed4 _Color;
			samplerCUBE _Cubemap;
			sampler2D _MainTex;
			sampler2D _WaveMap;
			sampler2D _RefractionTex;
			float4 _MainTex_ST;
			float4 _WaveMap_ST;
			float4 _RefractionTex_TexelSize;
			fixed _WaveXSpeed;
			fixed _WaveYSpeed;
			float _Distortion;	

            struct appdata
            {
                float4 vertex : POSITION;
				float3 normal : NORMAL;
				float4 tangent : TANGENT; 
				float4 texcoord : TEXCOORD0;
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
				float4 scrPos : TEXCOORD0;
				float4 uv : TEXCOORD1;
				float4 TtoW0 : TEXCOORD2;  
				float4 TtoW1 : TEXCOORD3;  
				float4 TtoW2 : TEXCOORD4; 
            };

            v2f vert (appdata v)
            {
                v2f o;
				o.pos = UnityObjectToClipPos(v.vertex);
				o.scrPos = ComputeGrabScreenPos(o.pos);
				o.uv.xy = TRANSFORM_TEX(v.texcoord, _MainTex);
				o.uv.zw = TRANSFORM_TEX(v.texcoord, _WaveMap);
				
				float3 worldPos = mul(unity_ObjectToWorld, v.vertex).xyz;  
				fixed3 worldNormal = UnityObjectToWorldNormal(v.normal);  
				fixed3 worldTangent = UnityObjectToWorldDir(v.tangent.xyz);  
				fixed3 worldBinormal = cross(worldNormal, worldTangent) * v.tangent.w; 
				
				o.TtoW0 = float4(worldTangent.x, worldBinormal.x, worldNormal.x, worldPos.x);  
				o.TtoW1 = float4(worldTangent.y, worldBinormal.y, worldNormal.y, worldPos.y);  
				o.TtoW2 = float4(worldTangent.z, worldBinormal.z, worldNormal.z, worldPos.z);  
				
				return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                float3 worldPos = float3(i.TtoW0.w, i.TtoW1.w, i.TtoW2.w);
				fixed3 viewDir = normalize(UnityWorldSpaceViewDir(worldPos));
				float2 speed = _Time.y * float2(_WaveXSpeed, _WaveYSpeed);
				
				// normal in tan_Space, Double sample imitate two layer effect
				fixed3 bump1 = UnpackNormal(tex2D(_WaveMap, i.uv.zw + speed)).rgb;
				fixed3 bump2 = UnpackNormal(tex2D(_WaveMap, i.uv.zw - speed)).rgb;
				fixed3 bump = normalize(bump1 + bump2);
				
				// offset in tan_Space
				float2 offset = bump.xy * _Distortion * _RefractionTex_TexelSize.xy;
                    // Related to Z: more deep, more distort
                    // bind with main camera, but without free free view
				i.scrPos.xy = offset * i.scrPos.z + i.scrPos.xy;
                    // activite free view - bug
				// i.scrPos.xy = offset * i.scrPos.xy;
				fixed3 refrCol = tex2D( _RefractionTex, i.scrPos.xy/i.scrPos.w).rgb;
				
				// Convert the normal to world space
				bump = normalize(half3(dot(i.TtoW0.xyz, bump), 
                    dot(i.TtoW1.xyz, bump), dot(i.TtoW2.xyz, bump)));
				fixed4 texColor = tex2D(_MainTex, i.uv.xy + speed);
				fixed3 reflDir = reflect(-viewDir, bump);
				fixed3 reflCol = texCUBE(_Cubemap, reflDir).rgb * texColor.rgb * _Color.rgb;
				
				fixed fresnel = pow(1 - saturate(dot(viewDir, bump)), 4);
				fixed3 finalColor = reflCol * fresnel + refrCol * (1 - fresnel);
				
				return fixed4(finalColor, 1);
            }
            ENDCG
        }
    }
    FallBack Off
}
