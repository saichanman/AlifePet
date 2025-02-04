Shader "Unlit/GridFloor"
{
    Properties
    {
        _Color ("Color", Color) = (1,1,1,1)
        _MainTex ("Grid Texture", 2D) = "white" {}
        _GridThickness ("Grid Thickness", Range(0, 0.01)) = 0.002 // グリッド線の太さ
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            sampler2D _MainTex;
            float4 _Color;
            float _GridThickness;

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                // グリッドの線を引くためのフラクション計算
                float2 grid = frac(i.uv / _GridThickness);
                
                // グリッドの線を引くためのスムーズなステップ計算
                float gridLine = step(0.99, grid.x) + step(0.99, grid.y);
                
                return lerp(_Color, float4(0.8, 0.8, 0.8, 1), gridLine); // グリッドラインをグレーに
            }
            ENDCG
        }
    }
    FallBack "Diffuse"
}