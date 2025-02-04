using UnityEngine;

public class CollisionFilter : MonoBehaviour
{
    [Header("対象のタグ")]
    public string targetTag = "ground"; // 判定対象のタグ名

    // 衝突判定（Is Trigger がオフの場合）
    private void OnCollisionEnter(Collision collision)
    {
        // 相手のタグをチェック
        if (collision.gameObject.CompareTag(targetTag))
        {
            Debug.Log($"{collision.gameObject.name}と衝突しました");
            // タグが一致する場合の処理をここに記述
        }
    }

    // トリガー判定（Is Trigger がオンの場合）
    private void OnTriggerEnter(Collider other)
    {
        // 相手のタグをチェック
        if (other.CompareTag(targetTag))
        {
            Debug.Log($"{other.gameObject.name}がトリガーに入りました");
            // タグが一致する場合の処理をここに記述
        }
    }
}