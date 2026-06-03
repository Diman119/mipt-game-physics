using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class SceneLauncher : MonoBehaviour {
    void Awake() {
        foreach (var b in GetComponentsInChildren<Button>()) {
            b.onClick.AddListener(delegate {
                var text = b.GetComponentInChildren<TextMeshProUGUI>().text;
                SceneManager.LoadSceneAsync(text.Substring(0, 3));
            });
        }
        Destroy(this);
    }
}
