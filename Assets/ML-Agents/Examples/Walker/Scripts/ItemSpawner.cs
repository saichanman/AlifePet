using System.Collections.Generic;
using UnityEngine;

public class ItemSpawner : MonoBehaviour
{
    [Header("Spawn Settings")]
    public GameObject foodPrefab;
    public GameObject toyPrefab;
    public int maxFoodItems = 5;
    public int maxToyItems = 1;
    public Vector3 spawnAreaSize = new Vector3(20f, 0f, 20f);

    public float foodRespawnDelay = 5f; // 餌の再スポーンまでの待機時間

    private List<GameObject> spawnedFoodItems = new List<GameObject>();
    private List<GameObject> spawnedToyItems = new List<GameObject>();
    private float foodRespawnTimer;

    public List<GameObject> SpawnedFoodItems => spawnedFoodItems;
    public List<GameObject> SpawnedToyItems => spawnedToyItems;

    void Start()
    {
        SpawnInitialItems();
    }

    void Update()
    {
        HandleFoodRespawn();
    }

    void SpawnInitialItems()
    {
        for (int i = 0; i < maxFoodItems; i++)
        {
            SpawnFood();
        }

        for (int i = 0; i < maxToyItems; i++)
        {
            SpawnToy();
        }
    }

    void HandleFoodRespawn()
    {
        if (spawnedFoodItems.Count < maxFoodItems)
        {
            foodRespawnTimer += Time.deltaTime;

            if (foodRespawnTimer >= foodRespawnDelay)
            {
                SpawnFood();
                foodRespawnTimer = 0f; // タイマーをリセット
            }
        }
    }

    public void SpawnFood()
    {
        if (spawnedFoodItems.Count >= maxFoodItems)
            return;

        Vector3 spawnPosition = GetRandomSpawnPosition();
        GameObject newFood = Instantiate(foodPrefab, spawnPosition, Quaternion.identity);
        spawnedFoodItems.Add(newFood);
    }

    public void SpawnToy()
    {
        if (spawnedToyItems.Count >= maxToyItems)
            return;

        Vector3 spawnPosition = GetRandomSpawnPosition();
        GameObject newToy = Instantiate(toyPrefab, spawnPosition, Quaternion.identity);
        spawnedToyItems.Add(newToy);
    }

    Vector3 GetRandomSpawnPosition()
    {
        Vector3 randomPosition = new Vector3(
            Random.Range(-spawnAreaSize.x / 2f, spawnAreaSize.x / 2f),
            0f,
            Random.Range(-spawnAreaSize.z / 2f, spawnAreaSize.z / 2f)
        );
        return randomPosition + transform.position;
    }

    public void RemoveFood(GameObject food)
    {
        if (spawnedFoodItems.Contains(food))
        {
            spawnedFoodItems.Remove(food);
            Destroy(food);
            foodRespawnTimer = 0f; // タイマーをリセット
        }
    }

    public void RemoveToy(GameObject toy)
    {
        if (spawnedToyItems.Contains(toy))
        {
            spawnedToyItems.Remove(toy);
            Destroy(toy);
        }
    }
}