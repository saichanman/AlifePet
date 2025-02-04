using UnityEngine;

public class ThrowableObject : MonoBehaviour
{
    public bool isFood = false; // 餌かどうかを判定するフラグ
    private Rigidbody rb;
    private Camera mainCamera;
    private bool isFollowingMouse = false;

    // 足場の制限範囲
    private Vector3 platformCenter = new Vector3(0, 0, 0); // 足場の中心
    private Vector3 platformSize = new Vector3(100, 0, 100); // 足場のサイズ

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("Rigidbody がアタッチされていません。");
            rb = gameObject.AddComponent<Rigidbody>();
        }

        mainCamera = Camera.main;
        if (mainCamera == null)
        {
            Debug.LogError("Main Camera が見つかりません。");
        }
    }

    void Update()
    {
        if (isFollowingMouse)
        {
            FollowMouse();

            // マウスクリックで放出
            if (Input.GetMouseButtonDown(0))
            {
                ThrowObject();
            }
        }
    }

    public void StartFollowingMouse()
    {
        isFollowingMouse = true;
        rb.isKinematic = true; // マウス追従中は物理演算を無効化
    }

    private void FollowMouse()
    {
        if (mainCamera == null) return;

        Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
        if (Physics.Raycast(ray, out RaycastHit hit))
        {
            Vector3 mousePosition = hit.point;
            mousePosition.y = 1; // 高さを固定
            transform.position = mousePosition;
        }
    }

    private void ThrowObject()
    {
        isFollowingMouse = false;
        rb.isKinematic = false; // 物理演算を再有効化

        // マウス位置から方向を計算して力を加える
        Vector3 throwDirection = CalculateThrowDirection();
        rb.AddForce(throwDirection * 500f); // 力の大きさは調整可能
    }

    private Vector3 CalculateThrowDirection()
    {
        Vector3 targetPosition = new Vector3(
            Mathf.Clamp(transform.position.x, platformCenter.x - platformSize.x / 2, platformCenter.x + platformSize.x / 2),
            0,
            Mathf.Clamp(transform.position.z, platformCenter.z - platformSize.z / 2, platformCenter.z + platformSize.z / 2)
        );

        return (targetPosition - transform.position).normalized;
    }
}