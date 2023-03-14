// ---------------------------- esp_ now -------------------------
void printMAC(const uint8_t * mac_addr) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success to " : "Delivery Fail to ");
  printMAC(mac_addr);
  Serial.println();
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  Serial.print(len);
  Serial.print(" bytes of data received from : ");
  printMAC(mac_addr);
  Serial.println();
  StaticJsonDocument<1000> root;
  String payload;
  
  // Process pairing request. Send some data straight away!
  memcpy(&pairingData, incomingData, sizeof(pairingData));
  Serial.print("Pairing request from: ");
  printMAC(mac_addr);
  Serial.print(" | Channel ");
  Serial.println(pairingData.channel);
  sendValuesEspNow();
  //addPeer(mac_addr);
}

void initESPNow() {
  Serial.print("Channel ");
  Serial.println(WiFi.channel());

  currentChannel = WiFi.channel();
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else {
    Serial.println("ESP-NOW initialised.");
  }

  if (esp_now_is_peer_exist(broadcastAddress))
  {
      esp_now_del_peer(broadcastAddress);
  }

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = WiFi.channel();
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
      Serial.println("Failed to add peer");
      return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}
