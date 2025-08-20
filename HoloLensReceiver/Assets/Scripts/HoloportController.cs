/***************************************************************************\

Module Name:  HoloportController.cs
Project:      HoloLensReceiver
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module manages the different active Holoports in the session.

\***************************************************************************/

using System.Collections.Generic;
using UnityEngine;

public class HoloportController : MonoBehaviour
{
    // Renderer object
    public GameObject HoloportPrefab;
    public List<string> DefaultServerIPAddresses = new List<string> { "127.0.0.1" };
    private Dictionary<string, GameObject> holoports = new Dictionary<string, GameObject>();

    void Start()
    {
        // Connect to all default server addresses
        foreach (string ipAddress in DefaultServerIPAddresses)
        {
            // Store connected addresses and HoloportPrefab instances
            GameObject newHoloport = Instantiate(HoloportPrefab, this.transform);
            HoloportReceiver newPointCloudReceiver = newHoloport.GetComponent<HoloportReceiver>();
            newPointCloudReceiver.ServerIPAddress = ipAddress;
            newPointCloudReceiver.IsServerIPAddressSet = true;
            holoports.Add(ipAddress, newHoloport);
        }
    }
}
