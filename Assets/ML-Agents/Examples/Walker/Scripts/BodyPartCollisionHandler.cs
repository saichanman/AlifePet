using UnityEngine;

public class BodyPartCollisionHandler : MonoBehaviour
{
    public QuadrupedAgent agent;

    private void Awake()
    {
        if (agent == null)
        {
            agent = GetComponentInParent<QuadrupedAgent>();
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (agent != null)
        {
            agent.HandleCollision(collision, this.gameObject);
        }
        else
        {
            Debug.LogWarning("エージェントの参照が設定されていません: " + gameObject.name);
        }
    }
}