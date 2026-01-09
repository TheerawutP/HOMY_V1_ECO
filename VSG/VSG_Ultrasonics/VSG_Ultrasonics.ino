#define deviceNum   5  //amount of used ultrasonic ss
#define ALM         15 //output pin when detected
#define RANGE_START 2 
#define RANGE_END   60
#define RANGE_END_AUX 100 //range for lateral ultrasonic ss
#define TIMEOUT     30000
#define DECROSSTALK 60   //period for each device trig-echo then move to another one

uint8_t rx_pin[deviceNum]  = {33, 14, 18, 4, 25};
uint8_t tx_pin[deviceNum]  = {32, 27, 19, 16, 26};
uint8_t LED_pin[deviceNum] = {12, 23, 22, 21, 5};

uint32_t duration[deviceNum];
uint32_t distanceRaw[deviceNum];
uint32_t distance[deviceNum];

// ----- Median Filter Buffer -----
uint32_t medBuf[deviceNum][3];
uint8_t medIndex[deviceNum] = {0};

// ------- Median function -------
uint32_t median3(uint32_t a, uint32_t b, uint32_t c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

void setup() {
  Serial.begin(115200);
  pinMode(ALM, OUTPUT);

  for (int i = 0; i < deviceNum; i++) {
    pinMode(rx_pin[i], INPUT);
    pinMode(tx_pin[i], OUTPUT);
    pinMode(LED_pin[i], OUTPUT);

    digitalWrite(tx_pin[i], LOW);
    digitalWrite(LED_pin[i], LOW);

    // init median buffer
    medBuf[i][0] = medBuf[i][1] = medBuf[i][2] = 999;
  }
}

void loop() {
  bool alert = false;

  for (int i = 0; i < deviceNum; i++) {

    // ------- Trigger -------
    digitalWrite(tx_pin[i], LOW);
    delayMicroseconds(2);
    digitalWrite(tx_pin[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(tx_pin[i], LOW);

    // ------- Read Echo -------
    duration[i] = pulseIn(rx_pin[i], HIGH, TIMEOUT);

    if (duration[i] == 0) {
      distanceRaw[i] = 999;
    } else {
      distanceRaw[i] = (duration[i] * 0.0343) / 2;
    }

    // ------- Median Buffer Update -------
    medBuf[i][medIndex[i]] = distanceRaw[i];
    medIndex[i] = (medIndex[i] + 1) % 3;

    // ------- Median Filter -------
    distance[i] = median3(medBuf[i][0], medBuf[i][1], medBuf[i][2]);


    // ------- Range Check -------
    bool inRange;

    if(i != 4){
      inRange = (distance[i] > RANGE_START && distance[i] < RANGE_END); 
    } else {
      inRange = (distance[i] > RANGE_START && distance[i] < RANGE_END_AUX); 
    }

    digitalWrite(LED_pin[i], inRange ? HIGH : LOW);
    if (inRange) alert = true;

    delay(DECROSSTALK);  
  }

  digitalWrite(ALM, alert ? HIGH : LOW);

  // ============================================================
  //  Plotter Output (distance[0]–distance[4])  
  // ============================================================
  Serial.print(distance[0]); Serial.print(",");
  Serial.print(distance[1]); Serial.print(",");
  Serial.print(distance[2]); Serial.print(",");
  Serial.print(distance[3]); Serial.print(",");
  Serial.println(distance[4]);
}
