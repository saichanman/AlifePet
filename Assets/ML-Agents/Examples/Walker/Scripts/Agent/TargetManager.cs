using UnityEngine;

public class TargetManager : MonoBehaviour
{
    public GameObject foodPrefab; // 食べ物のプレハブ
    public GameObject toyPrefab;  // おもちゃのプレハブ

    private GameObject currentTarget; // 現在のターゲット

    /// <summary>
    /// 食べ物をターゲットとして生成します。
    /// </summary>
    public void SpawnFood()
    {
        ClearTarget(); // 現在のターゲットを削除
        currentTarget = Instantiate(foodPrefab, GetRandomPosition(), Quaternion.identity);
    }

    /// <summary>
    /// おもちゃをターゲットとして生成します。
    /// </summary>
    public void SpawnToy()
    {
        ClearTarget(); // 現在のターゲットを削除
        currentTarget = Instantiate(toyPrefab, GetRandomPosition(), Quaternion.identity);
    }

    /// <summary>
    /// 現在のターゲットを削除してランダムな位置に新しいターゲットを生成します。
    /// </summary>
    public void SpawnRandomTarget()
    {
        ClearTarget(); // 現在のターゲットを削除
        // ランダムなターゲットとして食べ物かおもちゃを生成
        GameObject randomPrefab = Random.Range(0, 2) == 0 ? foodPrefab : toyPrefab;
        currentTarget = Instantiate(randomPrefab, GetRandomPosition(), Quaternion.identity);
    }

    /// <summary>
    /// 現在のターゲットを削除します。
    /// </summary>
    public void ClearTarget()
    {
        if (currentTarget != null)
        {
            Destroy(currentTarget);
            currentTarget = null;
        }
    }

    /// <summary>
    /// ランダムな位置を取得します。
    /// </summary>
    /// <returns>ランダムな座標</returns>
    private Vector3 GetRandomPosition()
    {
        float x = Random.Range(-40f, 40f);
        float z = Random.Range(-40f, 40f);
        return new Vector3(x, 0f, z);
    }
}