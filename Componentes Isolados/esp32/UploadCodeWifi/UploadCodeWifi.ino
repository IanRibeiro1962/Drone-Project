#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

const char* ssid = "SEU_SSID";         // Substitua pelo nome da sua rede Wi-Fi
const char* password = "SUA_SENHA";     // Substitua pela senha da sua rede Wi-Fi

WebServer server(80);

// Página HTML para upload do firmware
const char* uploadHTML = "<!DOCTYPE html>\
<html>\
  <head><meta charset='utf-8'><title>Update Firmware</title></head>\
  <body>\
    <form method='POST' action='/update' enctype='multipart/form-data'>\
      <input type='file' name='update'>\
      <input type='submit' value='Atualizar Firmware'>\
    </form>\
  </body>\
</html>";

void handleRoot() {
  server.send(200, "text/html", uploadHTML);
}

void handleUpdate() {
  HTTPUpload &upload = server.upload();
  
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("Iniciando atualização: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { // Inicializa com o tamanho máximo disponível
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    // Escreve os dados recebidos na memória flash
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) { // true define o tamanho para o progresso atual
      Serial.printf("Atualização concluída: %u bytes escritos\n", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
  }
  yield();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando...");

  // Conecta-se à rede Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando ao Wi-Fi...");
  }
  Serial.println("Wi-Fi conectado");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Configura as rotas do servidor web
  server.on("/", HTTP_GET, handleRoot);
  server.on("/update", HTTP_POST, [](){
      // Responde após a atualização e reinicia o dispositivo
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError() ? "Falha" : "Atualização OK"));
      delay(1000);
      ESP.restart();
    }, handleUpdate);

  server.begin();
  Serial.println("Servidor HTTP iniciado");
}

void loop() {
  server.handleClient();
}
