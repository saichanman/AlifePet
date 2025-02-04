using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RandomGridTerrain : MonoBehaviour
{
    public float gridHeight = 2f; // グリッドの高さの最大値
    public float gridSize = 1f;   // グリッドの大きさ（1ユニットを維持）
    public int gridResolution = 10; // グリッドの解像度（縦・横のセル数）

    private MeshFilter meshFilter;
    private Mesh mesh;
    private Vector3[] vertices;
    private Vector2[] uv; // UVマッピング用

    void Start()
    {
        meshFilter = GetComponent<MeshFilter>();
        if (meshFilter != null)
        {
            GenerateRandomGrid();
        }
    }

    void GenerateRandomGrid()
    {
        // メッシュの初期化
        mesh = new Mesh();
        meshFilter.mesh = mesh;

        // 頂点を作成
        vertices = new Vector3[(gridResolution + 1) * (gridResolution + 1)];
        uv = new Vector2[vertices.Length]; // UVマッピング用配列を初期化

        // グリッドを中央基準にするためにオフセットを計算
        float xOffset = (gridResolution * gridSize) / 2f;
        float zOffset = (gridResolution * gridSize) / 2f;

        for (int i = 0, z = 0; z <= gridResolution; z++)
        {
            for (int x = 0; x <= gridResolution; x++)
            {
                // ランダムな高さを設定し、PlatformのY座標を考慮
                float y = Random.Range(0f, gridHeight); 
                vertices[i] = new Vector3(x * gridSize - xOffset + 0.0f, 
                                          y, 
                                          z * gridSize - zOffset + 0.0f); // 各Platformのワールド位置を基準に頂点を設定

                // UVマッピングの設定: 0〜1の範囲でグリッドをマッピング
                uv[i] = new Vector2((float)x / gridResolution, (float)z / gridResolution);
                i++;
            }
        }

        // 三角形を作成
        int[] triangles = new int[gridResolution * gridResolution * 6];
        for (int ti = 0, vi = 0, z = 0; z < gridResolution; z++, vi++)
        {
            for (int x = 0; x < gridResolution; x++, ti += 6, vi++)
            {
                triangles[ti] = vi;
                triangles[ti + 1] = vi + gridResolution + 1;
                triangles[ti + 2] = vi + 1;
                triangles[ti + 3] = vi + 1;
                triangles[ti + 4] = vi + gridResolution + 1;
                triangles[ti + 5] = vi + gridResolution + 2;
            }
        }

        // メッシュに頂点と三角形、UVを設定
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.uv = uv; // UVマッピングを適用
        mesh.RecalculateNormals(); // 法線の再計算（ライティングのため）

        // コライダーの更新
        MeshCollider meshCollider = gameObject.GetComponent<MeshCollider>();
        if (meshCollider != null)
        {
            meshCollider.sharedMesh = mesh;
        }
    }
}