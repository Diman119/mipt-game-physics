using UnityEngine;
using UnityEngine.SceneManagement;

public class MenuLauncher : MonoBehaviour {
    public void ToMenu() => SceneManager.LoadSceneAsync("Launcher");
}
