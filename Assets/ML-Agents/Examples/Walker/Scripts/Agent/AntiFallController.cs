using UnityEngine;

public class AntiFallController : MonoBehaviour
{
    public Rigidbody targetRigidbody; // 上向きの力を加える対象 (エージェントの Rigidbody)
    public float upwardForce = 50f;   // 上向きの力の強さ
    public string groundTag = "ground"; // 地面のタグ

    private void OnTriggerEnter(Collider other)
    {
        // 地面に接触した場合
        if (other.CompareTag(groundTag))
        {
            // 衝突点を基準に上向きの力を加える
            Vector3 upwardDirection = Vector3.up;

            // 力を加える処理
            if (targetRigidbody != null)
            {
                targetRigidbody.AddForce(upwardDirection * upwardForce, ForceMode.Impulse);
                Debug.Log("Anti-fall force applied!");
            }
        }
    }
}