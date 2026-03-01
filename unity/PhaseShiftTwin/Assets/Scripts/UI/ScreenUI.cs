using TMPro;
using UnityEngine;

namespace UI
{
    public class ScreenUI : MonoBehaviour
    {
        [SerializeField] private TMP_Text _message;

        public void SetText(string msg)
        {
            _message.text = msg;
        }

        public void Toggle(bool toggle)
        {
            gameObject.SetActive(toggle);
        }
    }
}
