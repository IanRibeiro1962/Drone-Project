#include <WiFi.h>

const char* ssid = "IFPE";
const char* password = "";

WiFiServer server(8080);

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("Servidor iniciado.");
}

void loop() {
  // Verifica se há um novo cliente conectado
  WiFiClient client = server.available();

  if (client) {
    // Assim que um cliente se conecta, mostramos no monitor serial
    Serial.println("Novo cliente conectado.");

    // Enquanto o cliente estiver conectado
    while (client.connected()) {
      // Se houver dados vindos do cliente, podemos ler (opcional)
      if (client.available()) {
        String recebido = client.readStringUntil('\n');
        Serial.print("Recebido do cliente: ");
        Serial.println(recebido);
      }
      // Exemplo: envia uma string com dados periodicamente

      delay(1000);
    }

    // Quando o cliente desconectar
    Serial.println("Cliente desconectado.");
  }
}
