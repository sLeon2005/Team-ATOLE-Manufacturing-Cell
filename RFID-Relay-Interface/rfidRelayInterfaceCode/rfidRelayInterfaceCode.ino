#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ── LCD I2C ─────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ── Pines RC522 ─────────────────────────────
#define SS_PIN 8
#define RST_PIN 9

MFRC522 rfid(SS_PIN, RST_PIN);

// ── Relés ───────────────────────────────────
#define K1 3
#define K2 2
#define K3 A0
#define K4 5

// ── Usuarios registrados ────────────────────
struct Usuario {
  String uid;
  String nombre;
};

Usuario usuarios[] = {
  { "7F36B80C", "Sebastian Leon" },
  { "AAAAAAAA", "Kintia Negrete" },
  { "BDFD8E6E", "Jesus Zamora" },
  { "1A9A8CBE", "Francisco Hernandez" },
  { "3AE092BE", "Yudy Perez" }
};

const int numUsuarios = sizeof(usuarios) / sizeof(usuarios[0]);

// ── Anti rebote ─────────────────────────────
String ultimoUID = "";
unsigned long ultimoTiempo = 0;
const unsigned long cooldown = 2000;

// ────────────────────────────────────────────
String uidToString(MFRC522::Uid *uid) {

  String resultado = "";

  for (byte i = 0; i < uid->size; i++) {

    if (uid->uidByte[i] < 0x10) {
      resultado += "0";
    }

    resultado += String(uid->uidByte[i], HEX);
  }

  resultado.toUpperCase();

  return resultado;
}

// ────────────────────────────────────────────
void mostrarInicio() {

  lcd.clear();

  lcd.setCursor(5, 0);
  lcd.print("ATOLE");

  lcd.setCursor(2, 1);
  lcd.print("S.A. de C.V.");
}

// ────────────────────────────────────────────
void setRelays(byte valor) {

  digitalWrite(K1, bitRead(valor, 0));  // LSB
  digitalWrite(K2, bitRead(valor, 1));
  digitalWrite(K3, bitRead(valor, 2));
  digitalWrite(K4, bitRead(valor, 3));  // MSB
}

// ────────────────────────────────────────────
byte obtenerNibble(String uid) {

  if (uid == "7F36B80C") return 0b0001;  // Sebastian
  if (uid == "AAAAAAAA") return 0b0010;  // Kintia
  if (uid == "BDFD8E6E") return 0b0011;  // Jesus
  if (uid == "1A9A8CBE") return 0b0100;  // Francisco
  if (uid == "3AE092BE") return 0b0101;  // Yudy

  return 0b0000;  // inválido
}

// ────────────────────────────────────────────
void setup() {

  Serial.begin(9600);

  // ── Configuración relés ──────────────────
  pinMode(K1, OUTPUT);
  pinMode(K2, OUTPUT);
  pinMode(K3, OUTPUT);
  pinMode(K4, OUTPUT);

  // Estado inicial = 0000
  setRelays(0b0000);

  // ── LCD ──────────────────────────────────
  lcd.init();
  lcd.backlight();

  mostrarInicio();

  // ── RFID ─────────────────────────────────
  SPI.begin();
  rfid.PCD_Init();

  Serial.println("RFID system ready...");
}

// ────────────────────────────────────────────
void loop() {

  // Esperar tarjeta
  if (!rfid.PICC_IsNewCardPresent()) {
    return;
  }

  if (!rfid.PICC_ReadCardSerial()) {
    return;
  }

  String uidLeido = uidToString(&rfid.uid);

  // Anti rebote
  if (uidLeido == ultimoUID && millis() - ultimoTiempo < cooldown) {

    rfid.PICC_HaltA();
    return;
  }

  ultimoUID = uidLeido;
  ultimoTiempo = millis();

  Serial.print("UID detectado: ");
  Serial.println(uidLeido);

  // Buscar usuario
  bool encontrado = false;

  for (int i = 0; i < numUsuarios; i++) {

    if (uidLeido == usuarios[i].uid) {

      Serial.print("Bienvenido: ");
      Serial.println(usuarios[i].nombre);

      // ── Activar relés ────────────────────
      byte nibble = obtenerNibble(uidLeido);
      setRelays(nibble);

      // ── Mostrar usuario en LCD ───────────
      lcd.clear();

      lcd.setCursor(0, 0);
      lcd.print("User:");

      lcd.setCursor(0, 1);
      lcd.print(usuarios[i].nombre);

      encontrado = true;

      // Mantener 4 segundos
      delay(4000);

      // Regresar relés a 0000
      setRelays(0b0000);

      // Regresar pantalla inicial
      mostrarInicio();

      break;
    }
  }

  // ── Usuario inválido ─────────────────────
  if (!encontrado) {

    Serial.println("INVALID USER");

    // Relés = 0000
    setRelays(0b0000);

    lcd.clear();

    lcd.setCursor(5, 0);
    lcd.print("ACCESS");

    lcd.setCursor(5, 1);
    lcd.print("DENIED");

    // Esperar 4 segundos
    delay(4000);

    // Regresar pantalla inicial
    mostrarInicio();
  }

  Serial.println();

  rfid.PICC_HaltA();
}