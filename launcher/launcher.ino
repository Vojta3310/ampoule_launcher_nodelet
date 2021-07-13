// Globals
#define launch 13
#define beep 3

bool armed = false;

void setup()
{
  pinMode(launch, OUTPUT);
  pinMode(beep, OUTPUT);
  digitalWrite(launch, LOW);
  Serial.begin(115200); // Initialize serial connection to display distance readings
}

void loop()
{
  uint8_t c = baca_read_single();
  if (c == 0x39 && armed) {
    digitalWrite(launch, HIGH);
    delay(5000);   
    digitalWrite(launch, LOW);
    delay(500);
    armed = false;
  }else if (c == 0x38){
    armed = true;
    baca_protocol(6);
  }else if (c == 0x37){
    armed = false;
  }

  if (armed){
    digitalWrite(beep, HIGH);
    delay(1);   
    digitalWrite(beep, LOW);
  }
  delay(1);
}

// Baca protocol sends chars in hexadecimal for Garmin:
// b to initialize the sequence<
// payload size
// distance char by char
// crc of all previously send chars including initialization
void baca_protocol(uint8_t msg) {
  uint8_t crc = (uint8_t) 'b';
  uint8_t payload_size = 1;

  Serial.write('b');
  Serial.write(payload_size + 1);

  Serial.write(0x10); // Add msg id
  Serial.write(msg);

  crc += (payload_size + 1) + 0x10 + msg;

  Serial.write(crc);
}

uint8_t baca_read_single() {
  // Check if we receive a command from the computer
  if (Serial.available() > 2) {
    uint8_t crc = 0;
    uint8_t tmp_in;
    uint8_t cmd;

    tmp_in = Serial.read();
    if (tmp_in == 'b') {
      crc += tmp_in;
      tmp_in = Serial.read();
      if (true) {
        crc += tmp_in;
        cmd = Serial.read();
        crc += cmd;
        if (crc == Serial.read()) {
          return cmd;
        }
      }
    }
  }
  return 255;
}
