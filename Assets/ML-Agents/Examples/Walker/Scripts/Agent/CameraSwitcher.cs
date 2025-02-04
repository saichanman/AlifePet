using UnityEngine;

public class CameraSwitcher : MonoBehaviour
{
    public Camera mainCamera; // メインカメラ
    public Camera subCamera;  // サブカメラ (追従カメラ)

    private Camera activeCamera; // 現在アクティブなカメラ

    void Start()
    {
        // 初期設定
        if (mainCamera != null) mainCamera.enabled = true;
        if (subCamera != null) subCamera.enabled = false;

        activeCamera = mainCamera;
    }

    public void SwitchCamera()
    {
        if (mainCamera == null || subCamera == null) return;

        // カメラを切り替える
        if (activeCamera == mainCamera)
        {
            mainCamera.enabled = false;
            subCamera.enabled = true;
            activeCamera = subCamera;
        }
        else
        {
            mainCamera.enabled = true;
            subCamera.enabled = false;
            activeCamera = mainCamera;
        }
    }
}