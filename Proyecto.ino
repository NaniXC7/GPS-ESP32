#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// Credenciales Wi-Fi
const char* ssid = "Galaxy S24 Ultra E9ED";
const char* password = "daniel00";

// Parámetros de AWS IoT
const char* awsEndpoint = "alpl7xfmwms8n-ats.iot.us-east-1.amazonaws.com";
 // El endpoint de tu AWS IoT
const char* mqttTopic = "coordenadas/gps"; // El topic donde enviarás las coordenadas

// Certificados y claves para AWS IoT
// Debes proporcionar los archivos de certificados aquí o guardarlos en el sistema de archivos
const char* ca_cert = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n" \
"ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n" \
"b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\n" \
"MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n" \
"b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\n" \
"ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\n" \
"9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\n" \
"IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\n" \
"VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\n" \
"93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\n" \
"jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\n" \
"AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\n" \
"A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\n" \
"U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\n" \
"N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\n" \
"o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\n" \
"5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\n" \
"rqXRfboQnoZsG4q5WTP468SQvvG5\n" \
"-----END CERTIFICATE-----"; // Certificado raíz CA
const char* client_cert = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDWjCCAkKgAwIBAgIVAIrfqs3xh/IkNG2K/Drbl7SCcYqmMA0GCSqGSIb3DQEB\n" \
"CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t\n" \
"IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yNDEwMDkyMjQ4\n" \
"MDFaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh\n" \
"dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDwnRTqsSFqpgoJ+8GQ\n" \
"ZgrZLz1GXmQCze6CiOt3x8plvG1tv/f5o4VxO7PZhbcYoUORQ/Wje/1SZNkoftgN\n" \
"NIIjW7vtd88ApRyvPZ8QeUGVoLOGmoYAt/j4lF3zAFH7YL7nJrmAB2sg0cZ5n81U\n" \
"Pditey8MMwU69yf61Sdw/QA9tKdEXkdLuDoQGwKLV38f1a8MQaNZtUFw/67B3UHu\n" \
"/owOreHhNUrvhqagzcxwC9PZuFBTBnBnxRJ87KjxTPc15lA2+ImVMABPkUdyMjWd\n" \
"oXI3ypVumKhTPwNyrM8k/zMNwBqVlrB04YF3DTuP6Kth0IZKCLX3SeFQfOcPpJRL\n" \
"LFbFAgMBAAGjYDBeMB8GA1UdIwQYMBaAFC7pEkFCyxzUGuMcszBu2wWXJPDIMB0G\n" \
"A1UdDgQWBBTv3mIDfy0tnq1VFrU+y4l2RE0MBjAMBgNVHRMBAf8EAjAAMA4GA1Ud\n" \
"DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAU9PekvspjwMxSDrKoGxPVSQR\n" \
"9elJnJonmsf8zwBGjYywWm8+/OhOZlY+/TZGNErdxj7JwKa1WYmqNb4iVkSQg3aQ\n" \
"SNeMjAHzgxOZB+DigFaxQFPUCMZD2jykJ+cZBB50HP6E/B1XV1hYYUwfUgRG7+BA\n" \
"cvrjWDcT/4JITEWgaNWkFX8F99E5WRfPpIgtgXZ3xZ9Bn0j+XjUUMl3rhpY+0h3+\n" \
"wq0U1xjdv6j+kJVkbuXTxXxEyMI9lpDFS7OlER/bp/jMyzlgK53D8TSkq/nJUWIM\n" \
"SCJHhEMwwbV5ibDF/rfV6PPzvq46N8Azi94dtKSg5b4C0V+XTTeHbUNCJH7S0A==\n" \
"-----END CERTIFICATE-----"; // Certificado de cliente
const char* client_key = \
"-----BEGIN RSA PRIVATE KEY-----\n" \
"MIIEpAIBAAKCAQEA8J0U6rEhaqYKCfvBkGYK2S89Rl5kAs3ugojrd8fKZbxtbb/3\n" \
"+aOFcTuz2YW3GKFDkUP1o3v9UmTZKH7YDTSCI1u77XfPAKUcrz2fEHlBlaCzhpqG\n" \
"ALf4+JRd8wBR+2C+5ya5gAdrINHGeZ/NVD3YrXsvDDMFOvcn+tUncP0APbSnRF5H\n" \
"S7g6EBsCi1d/H9WvDEGjWbVBcP+uwd1B7v6MDq3h4TVK74amoM3McAvT2bhQUwZw\n" \
"Z8USfOyo8Uz3NeZQNviJlTAAT5FHcjI1naFyN8qVbpioUz8DcqzPJP8zDcAalZaw\n" \
"dOGBdw07j+irYdCGSgi190nhUHznD6SUSyxWxQIDAQABAoIBAQDmkJ+KhVHQMGtO\n" \
"RcCl7iJXG71LwQd06IC09R8uGHUKKBV0JZAp11f3uY4N0uQw5dcpTEYF8ro8SgcN\n" \
"lv7RKI/L8guS8kuaAp4KDrJtxsPh7awaP9hIuAILsaCVSqlEiOBl/+WhmXo+aMYX\n" \
"pdnL77fCBViACCPq7QuSgM7appWkiaxRwydMZ0zviAywtzkiSuKpUURY9xh/64DF\n" \
"heD3OLBHhVOkUXwknAYWgU7Zj7csWiu3HfLb6of7ADYlsHYLXdHnw9YVIk1Vjdvj\n" \
"AQLEvVjoWRH5R75A9PSaFVIVR+A4Hj4Ri4Oycfsl2aFVPxfLh9ZM53jhDF2TdZSx\n" \
"lLl1L9PBAoGBAPq14cVygYeQTcwAUiagmZFPLQxQDYjXELlBUK94wp6vDySBCdV9\n" \
"neavd4nTzkGxBihS0Dr4ayJJp45vyMdW7TEPCxIaDBeKk9XG1xrNow9VRhMDbqOJ\n" \
"6Jip6VoII96QdVkvOywP+hgwVn12ZyJGRXv9ANrtDyaMXY8fgdiWQ8gLAoGBAPWw\n" \
"qlJu5+9jvEU/aVuGER0zHi/xm0HupEaiojMeJM7IeccvXLNAXyaolU8WvCpnwtII\n" \
"ISqRV9XT4XAvTvuDlH5shMruB2WS+869LKR5gfNUlZ76ByiwWBcbbPzMZXMk69S0\n" \
"6Xxx/ZjKtKBMHgR3UDOiBnZdwjQEKJJp7wgbiA5vAoGAHyea9r5T+lSkVkVz7/jl\n" \
"HHmKT41jIS4YlezX6fZHuRBoAVFHt4/0n0YOxMHdT6x/52LjHXDhX1Fn0PtzJ9JV\n" \
"h1MnPQifmA1QcbBT6rDDIpSmMJasmQX6MX5r0RRR4MZyzrJf2AnkOj1Hgi/EgS3a\n" \
"VJ4MDSiISXIH7lEsAY4qviECgYEAgVUdY8V0HIVw/zbxPDen3KojEvPpWdzI51IH\n" \
"dkdA8wV/bYYWE1oofI40Au0h4mS6npue8eyOw0YFH2wX9LiAO8jeYoDIrgSgv2TA\n" \
"EljQOoplNMnDaPzjck6CodTq9qER7+fIHTVYXM0gJl3LU2iHdJavNjIG4CDsRp6z\n" \
"MJ5lkcsCgYB/a7wawaVGGKz6vr9Q0UHH0vnwQVQXlLJVZdU4itwcUE8k9qcieciT\n" \
"xbdzXwd903uSaZgGkL5/yj2HTN3Rgytvl8psTKiTJQgU8RHRd8Gdm7N1hu7+2R3a\n" \
"QGPscsOi2PRBQdlw0HdY2uRKkgccjg7cqGrhSmXuwBFokOm5lZjp/A==\n" \
"-----END RSA PRIVATE KEY-----"; // Llave privada del cliente

WiFiClientSecure net;
PubSubClient client(net);

HardwareSerial gpsSerial(2); // Usando UART2 (D16 = RX2, D17 = TX2)
LiquidCrystal_I2C lcd(0x27, 16, 2); // Dirección I2C para la pantalla LCD

TinyGPSPlus gps; // Instancia para el GPS

// Pines de los botones
const int btnGuardar = 12;
const int btnCambiar = 13;

// Pines de los LEDs
const int led1 = 26;
const int led2 = 4;
const int led3 = 5;
const int led4 = 18;

// Variables para la lógica de botones
int coordenadasIndex = 0;  // Posición de coordenada en uso
bool confirmarIndex = false; // Alternar entre "Sí" y "No"

// Para almacenar coordenadas
float coordenadas[4][2];  // Max 4 puntos [latitud, longitud]

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);  // GPS en UART2

  lcd.init();
  lcd.backlight();

  pinMode(btnGuardar, INPUT_PULLUP);
  pinMode(btnCambiar, INPUT_PULLUP);

  // Inicializar los LEDs
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);

  // Apagar todos los LEDs al inicio
  apagarLeds();

  // Conectar a Wi-Fi
  conectarWifi();

  // Configurar MQTT
  configurarAWSIoT();

  // Mostrar mensaje de inicio
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("PULSE UN BOTON");
  lcd.setCursor(1, 1);
  lcd.print("PARA CONTINUAR");

  // Esperar a que se presione un botón para continuar
  esperarBoton();
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());  // Decodificar datos del GPS
  }

  if (coordenadasIndex < 4) {
    obtenerCoordenada();  // Obtener coordenada del GPS

    delay(5000);  // Mostrar la coordenada por 1 segundo

    if (confirmarCoordenada()) {
      guardarCoordenada();
      coordenadasIndex++;
      encenderLed(coordenadasIndex);  // Encender el LED correspondiente
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Reingresando...");
      delay(1000);
    }

    if (coordenadasIndex == 4) {
      lcd.clear();
      lcd.print("Max. puntos");
      enviarCoordenadasAWS(); // Enviar las 4 coordenadas a AWS IoT
    }
  }

  // Mantener el cliente MQTT conectado
  client.loop();
}

// Función para conectarse al Wi-Fi
void conectarWifi() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CONECTANDO A LA");
  lcd.setCursor(5, 1);
  lcd.print("RED");


  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    lcd.print(".");
  }

  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("CONEXION");
  lcd.setCursor(3, 1);
  lcd.print("ESTABLECIDA");
  delay(1000);
}

// Configurar la conexión a AWS IoT
void configurarAWSIoT() {
  net.setCACert(ca_cert);
  net.setCertificate(client_cert);
  net.setPrivateKey(client_key);
  client.setServer(awsEndpoint, 8883);
}

// Función para enviar coordenadas a AWS IoT
void enviarCoordenadasAWS() {
  String payload = "{";
  for (int i = 0; i < 4; i++) {
    payload += "\"coordenada" + String(i+1) + "\": {\"latitud\": " + String(coordenadas[i][0], 6) + ", \"longitud\": " + String(coordenadas[i][1], 6) + "}";
    if (i < 3) payload += ",";
  }
  payload += "}";

  if (client.connect("ESP32Client")) {
    client.publish(mqttTopic, payload.c_str());
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Enviado AWS IoT");
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error AWS IoT");
  }
}

// Función para esperar a que se presione un botón
void esperarBoton() {
  while (digitalRead(btnGuardar) == HIGH && digitalRead(btnCambiar) == HIGH) {
    // Espera a que cualquiera de los botones sea presionado
    delay(100);  // Evitar que consuma demasiados recursos
  }
  lcd.clear();  // Limpiar pantalla una vez que se ha presionado un botón
}

// Obtener coordenada desde el GPS
void obtenerCoordenada() {
  // Esperar a que haya datos válidos del GPS
  while (!gps.location.isValid()) {
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("CARGANDO");
    lcd.setCursor(2, 1);
    lcd.print("COORDENADAS");
    delay(1000);  // Espera
  }

  // Almacenar latitud y longitud
  coordenadas[coordenadasIndex][0] = gps.location.lat();  // Latitud
  coordenadas[coordenadasIndex][1] = gps.location.lng();  // Longitud

  // Mostrar la latitud en la primera línea y la longitud en la segunda
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Lat:");
  lcd.print(coordenadas[coordenadasIndex][0], 6);  // Mostrar latitud
  lcd.setCursor(0, 1);
  lcd.print("Lng:");
  lcd.print(coordenadas[coordenadasIndex][1], 6);  // Mostrar longitud
}

// Confirmar si se desea guardar la coordenada
bool confirmarCoordenada() {
  confirmarIndex = false;  // Empezar en "No"
  lcd.clear();

  while (true) {
    lcd.setCursor(1, 0);
    lcd.print("DESEA GUARDAR?");
    lcd.setCursor(7, 1);
    lcd.print(confirmarIndex ? "SI" : "NO");

    if (digitalRead(btnCambiar) == LOW) {
      confirmarIndex = !confirmarIndex;  // Cambiar entre "Sí" y "No"
      delay(300);  // Debounce
    }

    if (digitalRead(btnGuardar) == LOW) {
      delay(300);  // Debounce
      return confirmarIndex;  // Retornar verdadero si selecciona "Sí"
    }
  }
}

// Guardar la coordenada en el array
void guardarCoordenada() {
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("GUARDANDO");
  lcd.setCursor(2, 1);
  lcd.print("COORDENADAS");
  delay(2000);
}

// Encender el LED correspondiente
void encenderLed(int index) {
  apagarLeds();  // Apagar todos los LEDs antes de encender el correcto

  if (index == 1) {
    digitalWrite(led1, HIGH);
  } else if (index == 2) {
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);
  } else if (index == 3) {
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);
    digitalWrite(led3, HIGH);
  } else if (index == 4) {
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);
    digitalWrite(led3, HIGH);
    digitalWrite(led4, HIGH);
  }
}

// Apagar todos los LEDs
void apagarLeds() {
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
  digitalWrite(led4, LOW);
}
