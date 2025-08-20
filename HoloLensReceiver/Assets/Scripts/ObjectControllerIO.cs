/***************************************************************************\

Module Name:  ObjectControllerIO.cs
Project:      HoloLensReceiver
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module allows controlling an object using keyboard keys and the mouse.

\***************************************************************************/

using UnityEngine;

public class ObjectControllerIO : MonoBehaviour
{
    public float TranslationSpeed = 1.0f;
    public float RotationSpeed = 2.0f;
    public bool EnableRotation = true; // Allow the mouse to control the rotation of the object

    private float xRotation = 0.0f; // Rotation around X (up/down)
    private float yRotation = 0.0f; // Rotation around Y (left/right)

    void Update()
    {
        // Get input from keyboard for movement
        if (Input.GetKey(KeyCode.A)) // Move left
        {
            transform.position += -transform.right * TranslationSpeed * Time.deltaTime;
        }
        if (Input.GetKey(KeyCode.D)) // Move right
        {
            transform.position += transform.right * TranslationSpeed * Time.deltaTime;
        }
        if (Input.GetKey(KeyCode.W)) // Move forward
        {
            transform.position += transform.forward * TranslationSpeed * Time.deltaTime;
        }
        if (Input.GetKey(KeyCode.S)) // Move backward
        {
            transform.position += -transform.forward * TranslationSpeed * Time.deltaTime;
        }
        if (Input.GetKey(KeyCode.Q)) // Move up
        {
            transform.position += Vector3.up * TranslationSpeed * Time.deltaTime;
        }
        if (Input.GetKey(KeyCode.E)) // Move down
        {
            transform.position += Vector3.down * TranslationSpeed * Time.deltaTime;
        }

        if (Input.GetKey(KeyCode.R)) // Toggle rotation
        {
            EnableRotation = !EnableRotation;
        }

        if (EnableRotation)
        {
            // Mouse-based object rotation (rotation around X and Y axes)
            float mouseX = Input.GetAxis("Mouse X");
            float mouseY = Input.GetAxis("Mouse Y");

            // Adjust rotation based on mouse movement and sensitivity
            xRotation -= mouseY * RotationSpeed; // Up/Down rotation (X axis)
            yRotation += mouseX * RotationSpeed; // Left/Right rotation (Y axis)

            // Clamp rotation to avoid object flipping
            xRotation = Mathf.Clamp(xRotation, -90.0f, 90.0f);

            // Apply the rotation to the object
            transform.rotation = Quaternion.Euler(xRotation, yRotation, 0.0f);
        }
    }
}