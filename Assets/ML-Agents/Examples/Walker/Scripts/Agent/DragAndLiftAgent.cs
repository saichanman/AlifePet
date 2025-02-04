using UnityEngine;

public class DragAndLiftAgent : MonoBehaviour
{
    private Rigidbody rb;
    private bool isDragging = false;
    private Vector3 offset;
    private Camera mainCamera;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        mainCamera = Camera.main; // メインカメラを取得
    }

    void Update()
    {
        // マウスクリックしてドラッグ開始
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit) && hit.transform == transform)
            {
                isDragging = true;
                offset = transform.position - GetMouseWorldPosition();
                rb.isKinematic = true; // ドラッグ中は物理挙動を停止
            }
        }

        // マウスクリックを離してドラッグ終了
        if (Input.GetMouseButtonUp(0) && isDragging)
        {
            isDragging = false;
            rb.isKinematic = false; // ドラッグ終了で物理挙動を再開
        }
    }

    void FixedUpdate()
    {
        if (isDragging)
        {
            // エージェントの位置をマウスのワールド座標に合わせる
            Vector3 targetPosition = GetMouseWorldPosition() + offset;
            rb.MovePosition(targetPosition);
        }
    }

    private Vector3 GetMouseWorldPosition()
    {
        // マウス位置をワールド座標に変換
        Vector3 mouseScreenPosition = Input.mousePosition;
        mouseScreenPosition.z = mainCamera.WorldToScreenPoint(transform.position).z; // 深度を維持
        return mainCamera.ScreenToWorldPoint(mouseScreenPosition);
    }
}