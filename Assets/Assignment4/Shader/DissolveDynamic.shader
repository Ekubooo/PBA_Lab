// Upgrade NOTE: replaced '_Object2World' with 'unity_ObjectToWorld'
// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

Shader "DissolveDynamic"
{
    Properties
    {
        _BurnAmount ("Burn Amount", Range(0.0, 1.0)) = 0.0
		_LineWidth ("Burn Line Width", Range(0.0, 1.0)) = 0.1
		_Speed ("Dissolve Speed", Range(0.0, 1.5)) = 0.5
		_MainTex ("Base (RGB)", 2D) = "white" {}
		_BumpMap ("Normal Map", 2D) = "bump" {}
		_BurnFirstColor ("Burn First Color", Color) = (1, 0, 0, 1)
		_BurnSecondColor ("Burn Second Color", Color) = (1, 0, 0, 1)
		_BurnMap ("Burn Map", 2D) = "white"{}
    }
    SubShader
    {
        Tags {"RenderType" = "Opaque"}
        Tags {"Queue" = "Geometry"}

        Pass
        {
            Tags {"LightMode" = "ForwardBase"}
            Cull Off

            CGPROGRAM
            #include "UnityCG.cginc"
            #include "Lighting.cginc"
			#include "AutoLight.cginc"
            #pragma multi_compile_fwdbase
            #pragma vertex vert
            #pragma fragment frag

            sampler2D _MainTex;
            float4 _MainTex_ST;
			sampler2D _BumpMap;
			float4 _BumpMap_ST;
			sampler2D _BurnMap;
			float4 _BurnMap_ST;

            fixed _BurnAmount;
			fixed _LineWidth;
			fixed4 _BurnFirstColor;
			fixed4 _BurnSecondColor;
			float _Speed;

            struct a2v
            {
                float4 vertex : POSITION;
				float3 normal : NORMAL;
				float4 tangent : TANGENT;
				float4 texcoord : TEXCOORD0;
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
				float2 uvMainTex : TEXCOORD0;
				float2 uvBumpMap : TEXCOORD1;
				float2 uvBurnMap : TEXCOORD2;
				float3 lightDir : TEXCOORD3;
				float3 worldPos : TEXCOORD4;
				SHADOW_COORDS(5)
            };

            v2f vert (a2v v)
            {
                v2f o;
				o.pos = UnityObjectToClipPos(v.vertex);
				
				o.uvMainTex = TRANSFORM_TEX(v.texcoord, _MainTex);
				o.uvBumpMap = TRANSFORM_TEX(v.texcoord, _BumpMap);
				o.uvBurnMap = TRANSFORM_TEX(v.texcoord, _BurnMap);
				
				TANGENT_SPACE_ROTATION;
  				o.lightDir = mul(rotation, ObjSpaceLightDir(v.vertex)).xyz;
  				o.worldPos = mul(unity_ObjectToWorld, v.vertex).xyz;
  				
  				TRANSFER_SHADOW(o);
				return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
				// texture animations
                // fixed3 burn = tex2D(_BurnMap, i.uvBurnMap + _Time.y * _Speed).rgb;
                fixed3 burn = tex2D(_BurnMap, i.uvBurnMap).rgb;

				// adding speed
				// float dpA = (_Time.y * _Speed) % 2;
				// float DissolveParameter = dpA > 1 ? 1 - dpA % 1 : dpA;
				float DissolveParameter = (_Time.y * _Speed) % 4;
				if(DissolveParameter > 1 && DissolveParameter <= 2)
					DissolveParameter = 1;
				else if(DissolveParameter > 2 && DissolveParameter <= 3)
					DissolveParameter = 1 - (DissolveParameter % 2) ;
				else if(DissolveParameter > 3 && DissolveParameter <= 4)
					DissolveParameter = 0;
				clip(burn.r - DissolveParameter);	// round play

				float3 tangentLightDir = normalize(i.lightDir);
				fixed3 tangentNormal = UnpackNormal(tex2D(_BumpMap, i.uvBumpMap));
				
				fixed3 albedo = tex2D(_MainTex, i.uvMainTex).rgb;
				
				fixed3 ambient = UNITY_LIGHTMODEL_AMBIENT.xyz * albedo;
				
				fixed3 diffuse = _LightColor0.rgb * albedo 
                    * max(0, dot(tangentNormal, tangentLightDir));

				// fixed t = 1 - smoothstep(0.0, _LineWidth, burn.r - DissolveParameter);
				fixed t = 1 - smoothstep(0.0, _LineWidth, burn.r - DissolveParameter);
				fixed3 burnColor = lerp(_BurnFirstColor, _BurnSecondColor, t);
				burnColor = pow(burnColor, 5);
				
				UNITY_LIGHT_ATTENUATION(atten, i, i.worldPos);
				fixed3 finalColor = lerp(ambient + diffuse * atten, 
					burnColor, t * step(0.0001, DissolveParameter));

				return fixed4(finalColor, 1);
            }
            ENDCG
        }

        // Shadow Pass
        Pass 
        {
			Tags {"LightMode" = "ShadowCaster"}
			
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			#pragma multi_compile_shadowcaster
			
			#include "UnityCG.cginc"
			
			fixed _BurnAmount;
			sampler2D _BurnMap;
			float4 _BurnMap_ST;
			float _Speed;
			
			struct v2f 
            {
				V2F_SHADOW_CASTER;
				float2 uvBurnMap : TEXCOORD1;
			};
			
			v2f vert(appdata_base v) 
            {
				v2f o;
				TRANSFER_SHADOW_CASTER_NORMALOFFSET(o)
				o.uvBurnMap = TRANSFORM_TEX(v.texcoord, _BurnMap);
				return o;
			}
			
			fixed4 frag(v2f i) : SV_Target 
            {
				fixed3 burn = tex2D(_BurnMap, i.uvBurnMap).rgb;

				// float dpA = (_Time.y * _Speed) % 2;
				// float DissolveParameter = dpA > 1 ? 1 - dpA % 1 : dpA;
				float DissolveParameter = (_Time.y * _Speed) % 4;
				if(DissolveParameter > 1 && DissolveParameter <= 2)
					DissolveParameter = 1;
				else if(DissolveParameter > 2 && DissolveParameter <= 3)
					DissolveParameter = 1 - (DissolveParameter % 2) ;
				else if(DissolveParameter > 3 && DissolveParameter <= 4)
					DissolveParameter = 0;
				// clip(burn.r - DissolveParameter);
				clip(burn.r - DissolveParameter);

				SHADOW_CASTER_FRAGMENT(i)
			}
			ENDCG
		}
    }
    FallBack "Diffuse"
}
