#include <Arduino.h>
#include <TFT_eSPI.h>
#include <esp_heap_caps.h>
#include <esp32-hal-psram.h>

// ==================== CONFIGURATION ====================

// Serial Communication
#define SERIAL_BAUD 921600       // Baud rate - must match RP2040 (default: 921600)
#define RXD2 22                  // ESP32 RX pin (GPIO22) - connects to RP2040 TX

// Measurement Settings
#define NUM_SAMPLES 5000         // Number of samples per measurement - must match RP2040
#define SPEED_OF_SOUND 330       // Speed of sound in m/s (330=air, ~1500=water)
#define SAMPLE_TIME 1.554e-6     // Time per sample in seconds - must match RP2040 setup

// Display Settings
#define COLUMN_WIDTH 2           // Pixels per measurement (1-4, default: 2)
                                 // 1=320 columns (slow scroll), 2=160 columns, 4=80 columns (fast scroll)

// Normalization Settings
#define NORM_DEADZONE_M 0.30     // Exclude first X meters from auto-gain (0.3-0.7 typical)
                                 // Prevents sensor ringing from affecting contrast

// ==================== CYD CONFIGURATION ====================
#define WATERFALL_WIDTH 320
#define WATERFALL_HEIGHT 240
#define WATERFALL_COLUMNS (WATERFALL_WIDTH / COLUMN_WIDTH)
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define TFT_BL 21
#define TXD2 17

// Packet structure from RP2040
#define PACKET_SIZE (1 + 6 + 2 * NUM_SAMPLES + 1)  // header + metadata + samples + checksum

// ==================== GLOBAL VARIABLES ====================
TFT_eSPI tft = TFT_eSPI();

// Waterfall buffer holds quantized (0-255) amplitudes for each pixel
uint8_t *waterfallRaw = nullptr;  // dynamically allocated [WATERFALL_WIDTH * WATERFALL_HEIGHT]
uint16_t currentColumn = 0;  // Index where next column will be written
uint16_t filledColumnCount = 0;
bool bufferFilled = false;

// Running statistics for dynamic level computation (mean / sigma) based on quantized samples
uint64_t waterfallSum = 0;
uint64_t waterfallSumSquares = 0;

uint16_t currentDepthIndex = 0;  // Sample index (0-5000)
float currentDepth = 0;  // Depth in cm (for display)

// Depth display area (top-left, excluded from waterfall rendering)
#define DEPTH_DISPLAY_X 0    // X position (left edge)
#define DEPTH_DISPLAY_Y 0    // Y position (top)
#define DEPTH_DISPLAY_WIDTH 100  // Width of depth display area
#define DEPTH_DISPLAY_HEIGHT 30  // Height of depth display area

// Scale label dimensions (left side, excluded from waterfall rendering)
#define SCALE_LABEL_X 0      // X position (left edge)
#define SCALE_LABEL_WIDTH 32  // Width of each label
#define SCALE_LABEL_HEIGHT 8  // Height of each label (font size 1)

// Maximum number of scale labels
#define MAX_SCALE_LABELS 10
struct ScaleLabelRect {
  int y;        // Y position
  int width;    // Width
  int height;   // Height
};
ScaleLabelRect scaleLabelRects[MAX_SCALE_LABELS];
int numScaleLabels = 0;

// Color palette: PyQtGraph 'cyclic' gradient (256 RGB triplets)
uint16_t colorMap[256];
static const uint8_t CYCLIC_GRADIENT[256][3] = {
  {255,0,4},{255,0,13},{255,0,17},{255,0,25},{255,0,30},{255,0,38},{255,0,43},{255,0,47},
  {255,0,55},{255,0,60},{255,0,68},{255,0,72},{255,0,76},{255,0,85},{255,0,89},{255,0,98},
  {255,0,102},{255,0,106},{255,0,115},{255,0,119},{255,0,128},{255,0,132},{255,0,136},{255,0,144},
  {255,0,149},{255,0,157},{255,0,162},{255,0,170},{255,0,174},{255,0,179},{255,0,187},{255,0,191},
  {255,0,200},{255,0,204},{255,0,208},{255,0,217},{255,0,221},{255,0,230},{255,0,234},{255,0,238},
  {255,0,247},{255,0,251},{251,0,255},{246,1,254},{242,0,255},{234,0,255},{230,0,255},{221,0,255},
  {217,0,255},{212,0,255},{204,0,255},{200,0,255},{191,0,255},{187,0,255},{179,0,255},{174,0,255},
  {170,0,255},{162,0,255},{157,0,255},{149,1,254},{144,0,255},{140,0,255},{132,0,255},{128,0,255},
  {119,0,255},{115,0,255},{111,0,255},{102,0,255},{98,0,255},{89,0,255},{85,0,255},{81,0,255},
  {72,0,255},{68,0,255},{60,1,254},{56,1,254},{51,0,255},{43,0,255},{38,0,255},{30,0,255},
  {25,0,255},{17,0,255},{13,0,255},{8,0,255},{0,0,255},{0,4,255},{0,13,255},{0,17,255},
  {0,21,255},{0,30,255},{1,35,254},{1,43,254},{0,47,255},{0,51,255},{0,60,255},{0,64,255},
  {0,72,255},{0,76,255},{0,81,255},{0,89,255},{0,94,255},{0,102,255},{0,106,255},{0,115,255},
  {0,119,255},{0,123,255},{1,132,254},{1,136,254},{0,145,255},{0,149,255},{0,153,255},{0,162,255},
  {0,166,255},{0,174,255},{0,179,255},{0,183,255},{0,191,255},{0,195,255},{0,204,255},{0,208,255},
  {0,213,255},{0,221,255},{1,224,254},{1,233,254},{0,238,255},{0,242,255},{0,251,255},{0,255,255},
  {0,255,251},{0,255,247},{0,255,242},{0,255,234},{0,255,229},{0,255,225},{0,255,217},{0,255,213},
  {0,255,204},{0,255,200},{0,255,195},{0,255,187},{0,255,183},{0,255,174},{0,255,170},{0,255,166},
  {0,255,157},{0,255,153},{0,255,145},{0,255,140},{0,255,136},{0,255,128},{0,255,123},{0,255,115},
  {0,255,110},{0,255,106},{0,255,98},{0,255,94},{0,255,85},{0,255,81},{0,255,72},{0,255,68},
  {0,255,64},{0,255,55},{0,255,51},{0,255,42},{0,255,38},{0,255,34},{0,255,26},{0,255,21},
  {0,255,13},{0,255,8},{0,255,4},{4,255,0},{9,255,0},{17,255,0},{21,255,0},{26,255,0},
  {34,255,0},{38,255,0},{47,255,0},{51,255,0},{60,255,0},{64,255,0},{68,255,0},{77,255,0},
  {81,255,0},{89,255,0},{94,255,0},{98,255,0},{106,255,0},{111,255,0},{119,255,0},{123,255,0},
  {128,255,0},{136,255,0},{140,255,0},{149,255,0},{153,255,0},{157,255,0},{166,255,0},{170,255,0},
  {179,255,0},{183,255,0},{187,255,0},{195,255,0},{200,255,0},{208,255,0},{212,255,0},{221,255,0},
  {225,255,0},{229,255,0},{238,255,0},{242,255,0},{251,255,0},{255,255,0},{255,251,0},{255,242,0},
  {255,238,0},{255,230,0},{255,225,0},{255,221,0},{255,213,0},{255,208,0},{255,200,0},{255,196,0},
  {255,191,0},{255,183,0},{255,179,0},{255,170,0},{255,166,0},{255,162,0},{255,153,0},{255,149,0},
  {255,140,0},{255,136,0},{255,128,0},{255,123,0},{255,119,0},{255,111,0},{255,106,0},{255,98,0},
  {255,94,0},{255,89,0},{255,81,0},{255,77,0},{255,68,0},{255,64,0},{255,60,0},{255,51,0},
  {255,47,0},{255,38,0},{255,34,0},{255,30,0},{255,21,0},{255,17,0},{255,8,0},{255,4,0}
};

// ==================== FUNCTION DECLARATIONS ====================
void initColorMap();
uint16_t mapValueToColor(uint8_t value);
bool readPacket(uint16_t &depth, float &temperature, float &driveVoltage, uint16_t *samples);
void downsampleToColumn(uint16_t *samples, uint16_t *column);
void drawWaterfall();
void drawDepthDisplay();
void drawVerticalScale();

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);  // USB Serial for debug
  delay(1000);

  Serial.println("\n\n=== CYD Echo Display ===");
  Serial.printf("Serial RX on GPIO22 at %d baud\n", SERIAL_BAUD);

  // CRITICAL: Increase RX buffer size BEFORE begin()
  // Default is only 256 bytes, we need at least 20KB to buffer 2 packets
  Serial2.setRxBufferSize(32768);  // 32KB buffer

  // Configure Serial2 with GPIO22 as RX
  Serial2.begin(SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2);

  Serial.println("Serial2 initialized on GPIO22 with 32KB RX buffer");

  // Enable backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);  // Turn on backlight

  // Initialize display
  tft.init();
  tft.setRotation(1);  // Landscape mode - fills entire screen!
  tft.setSwapBytes(true);  // Ensure pushImage interprets 16-bit colors correctly

  tft.fillScreen(TFT_BLACK);
  delay(500);

  Serial.printf("Display initialized: rotation=1, size=%dx%d\n", tft.width(), tft.height());

  // Initialize color map
  initColorMap();

  // Allocate waterfall buffer (prefer PSRAM if available)
  size_t waterfallBytes = WATERFALL_COLUMNS * WATERFALL_HEIGHT * sizeof(uint8_t);
  if (psramFound()) {
    waterfallRaw = static_cast<uint8_t *>(heap_caps_malloc(waterfallBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (waterfallRaw) {
      Serial.println("Waterfall buffer allocated in PSRAM.");
    }
  }
  if (!waterfallRaw) {
    waterfallRaw = static_cast<uint8_t *>(heap_caps_malloc(waterfallBytes, MALLOC_CAP_8BIT));
    if (waterfallRaw) {
      Serial.println("Waterfall buffer allocated in internal RAM.");
    }
  }
  if (!waterfallRaw) {
    Serial.println("ERROR: Unable to allocate waterfall buffer!");
    while (true) {
      delay(1000);
    }
  }

  memset(waterfallRaw, 0, waterfallBytes);
  waterfallSum = 0;
  waterfallSumSquares = 0;
  currentColumn = 0;
  filledColumnCount = 0;
  bufferFilled = false;

  // No sprite allocation needed - depth display uses fixed header area

  // Draw initial UI
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.drawString("CYD Echo Display", 10, 80);
  tft.setTextSize(1);
  tft.drawString("Waiting for RP2040...", 10, 110);

  Serial.printf("Display ready: %dx%d\n", tft.width(), tft.height());
  Serial.println("Listening for packets on GPIO22...\n");
}

// ==================== MAIN LOOP ====================
void loop() {
  static uint16_t samples[NUM_SAMPLES];
  uint16_t depth;
  float temperature, driveVoltage;

  if (readPacket(depth, temperature, driveVoltage, samples)) {
    // Store depth values
    currentDepthIndex = depth;  // Sample index (0-5000)
    currentDepth = depth * (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2;  // cm

    // Downsample 5000 samples to waterfall height (240 pixels vertically)
    uint16_t downsampledColumn[WATERFALL_HEIGHT];
    downsampleToColumn(samples, downsampledColumn);

    // Add to waterfall buffer at current column (newest data)
    uint16_t writeColumn = currentColumn;
    uint8_t *columnPtr = waterfallRaw + writeColumn * WATERFALL_HEIGHT;
    if (bufferFilled) {
      for (int y = 0; y < WATERFALL_HEIGHT; y++) {
        uint8_t oldValue = columnPtr[y];
        waterfallSum -= oldValue;
        waterfallSumSquares -= static_cast<uint64_t>(oldValue) * oldValue;
      }
    }

    for (int y = 0; y < WATERFALL_HEIGHT; y++) {
      uint16_t newValue = downsampledColumn[y];
      uint16_t shifted = newValue >> 4;  // Scale 16-bit (0-65535) to 12-bit (0-4095)
      uint8_t quantized = (shifted > 255) ? 255 : static_cast<uint8_t>(shifted);  // Clamp to 8-bit
      columnPtr[y] = quantized;
      waterfallSum += quantized;
      waterfallSumSquares += static_cast<uint64_t>(quantized) * quantized;
    }

    currentColumn = (currentColumn + 1) % WATERFALL_COLUMNS;  // Move to next column, wrap around

    if (!bufferFilled) {
      if (filledColumnCount < WATERFALL_COLUMNS) {
        filledColumnCount++;
      }
      if (filledColumnCount >= WATERFALL_COLUMNS) {
        bufferFilled = true;
        filledColumnCount = WATERFALL_COLUMNS;
      }
    }

    // Draw waterfall display
    drawWaterfall();

    // Draw overlays (scale first, then depth display to ensure depth is on top)
    drawVerticalScale();
    drawDepthDisplay();
  }
}

// ==================== PACKET READING ====================
bool readPacket(uint16_t &depth, float &temperature, float &driveVoltage, uint16_t *samples) {
  static uint8_t buffer[PACKET_SIZE];
  static bool synced = false;
  static int failedChecksums = 0;

  // Search for sync pattern: 0xFF 0xFF 0xAA
  bool foundSync = false;
  while (Serial2.available() > 0 && !foundSync) {
    uint8_t b = Serial2.read();

    if (b == 0xFF) {
      // Check if next byte is also 0xFF
      if (Serial2.peek() == 0xFF) {
        Serial2.read();  // consume second 0xFF
        // Check if next byte is 0xAA
        if (Serial2.peek() == 0xAA) {
          Serial2.read();  // consume 0xAA
          foundSync = true;
          break;
        }
      }
    }
  }

  if (!foundSync) {
    return false;  // No sync pattern found yet
  }

  // Read payload (6 bytes metadata + 10000 bytes samples)
  size_t payloadSize = 6 + 2 * NUM_SAMPLES;

  // Use readBytes with timeout (it waits internally)
  Serial2.setTimeout(200);  // 200ms timeout for readBytes
  size_t bytesRead = Serial2.readBytes(buffer, payloadSize + 1);  // +1 for checksum

  if (bytesRead != payloadSize + 1) {
    Serial.printf("Incomplete packet: expected %d, got %d\n", payloadSize + 1, bytesRead);
    synced = false;
    return false;
  }

  // Verify checksum
  uint8_t calcChecksum = 0;
  for (size_t i = 0; i < payloadSize; i++) {
    calcChecksum ^= buffer[i];
  }

  if (calcChecksum != buffer[payloadSize]) {
    failedChecksums++;
    if (failedChecksums % 10 == 0) {
      Serial.printf("Checksum mismatch #%d: expected 0x%02X, got 0x%02X (flush buffer)\n",
                    failedChecksums, calcChecksum, buffer[payloadSize]);
    }
    // Flush buffer to resync
    while (Serial2.available() > 0) {
      Serial2.read();
      delayMicroseconds(10);
    }
    synced = false;
    return false;
  }

  if (!synced) {
    Serial.println("✓✓✓ SYNCED! Valid packet received!");
    synced = true;
    failedChecksums = 0;
  }

  // Parse metadata (big-endian)
  depth = (buffer[0] << 8) | buffer[1];
  int16_t tempScaled = (buffer[2] << 8) | buffer[3];
  uint16_t vDrvScaled = (buffer[4] << 8) | buffer[5];

  temperature = tempScaled / 100.0f;
  driveVoltage = vDrvScaled / 100.0f;

  // Parse samples (big-endian)
  for (int i = 0; i < NUM_SAMPLES; i++) {
    int idx = 6 + i * 2;
    samples[i] = (buffer[idx] << 8) | buffer[idx + 1];
  }

  return true;
}

// ==================== DOWNSAMPLING ====================
void downsampleToColumn(uint16_t *samples, uint16_t *column) {
  // Downsample 5000 samples to 200 pixels (vertical)
  // Each pixel represents ~25 samples (5000 / 200)
  const int samplesPerPixel = NUM_SAMPLES / WATERFALL_HEIGHT;

  for (int y = 0; y < WATERFALL_HEIGHT; y++) {
    uint32_t blockSum = 0;
    int startIdx = y * samplesPerPixel;

    for (int i = 0; i < samplesPerPixel; i++) {
      blockSum += samples[startIdx + i];
    }

    uint16_t average = static_cast<uint16_t>(blockSum / samplesPerPixel);

    // FLIP VERTICALLY: sample 0 (near) at bottom, sample 4999 (far) at top
    int displayY = WATERFALL_HEIGHT - 1 - y;
    column[displayY] = average;
  }
}

void initColorMap() {
  for (int i = 0; i < 256; ++i) {
    const uint8_t r = CYCLIC_GRADIENT[i][0];
    const uint8_t g = CYCLIC_GRADIENT[i][1];
    const uint8_t b = CYCLIC_GRADIENT[i][2];
    colorMap[i] = tft.color565(r, g, b);
  }
}

uint16_t mapValueToColor(uint8_t value) {
  // Value is already 8-bit (0-255)
  return colorMap[value];
}

// ==================== RENDERING ====================
void drawWaterfall() {
  // Full redraw every frame to show scrolling effect.
  // Colors are computed dynamically based on mean±2σ across the buffered data,
  // mirroring the behaviour of the Python reference app.
  static uint16_t lineBuffer[WATERFALL_WIDTH];

  if (!waterfallRaw) {
    return;
  }

  size_t validColumns = bufferFilled ? WATERFALL_COLUMNS : filledColumnCount;
  if (validColumns == 0) {
    tft.fillRect(0, 0, WATERFALL_WIDTH, WATERFALL_HEIGHT, TFT_BLACK);
    return;
  }

  // Calculate statistics excluding dead zone (sensor ringing area)
  // Note: Y-axis is inverted - Y=0 is far (max depth), Y=240 is near (0m)
  const float SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2;  // cm per sample
  const float MAX_DEPTH_M = (NUM_SAMPLES * SAMPLE_RESOLUTION) / 100.0;
  const int deadzonePixels = (NORM_DEADZONE_M / MAX_DEPTH_M) * WATERFALL_HEIGHT;
  const int deadzoneStartY = WATERFALL_HEIGHT - deadzonePixels;  // Exclude bottom (near sensor)

  uint64_t validSum = 0;
  uint64_t validSumSquares = 0;
  size_t validPixelCount = 0;

  // Recalculate statistics excluding dead zone (bottom part = near sensor)
  for (size_t col = 0; col < validColumns; col++) {
    size_t bufferCol = bufferFilled ? (currentColumn + col) % WATERFALL_COLUMNS : col;
    uint8_t *columnPtr = waterfallRaw + bufferCol * WATERFALL_HEIGHT;

    for (int y = 0; y < deadzoneStartY; y++) {  // Only pixels ABOVE deadzone (far from sensor)
      uint8_t value = columnPtr[y];
      validSum += value;
      validSumSquares += static_cast<uint64_t>(value) * value;
      validPixelCount++;
    }
  }

  const double mean = (validPixelCount > 0) ? static_cast<double>(validSum) / validPixelCount : 0.0;
  const double meanSquares = (validPixelCount > 0) ? static_cast<double>(validSumSquares) / validPixelCount : 0.0;
  double variance = meanSquares - mean * mean;
  if (variance < 0.0) {
    variance = 0.0;
  }
  const double sigma = sqrt(variance);
  double minLevel = mean - 2.0 * sigma;
  double maxLevel = mean + 2.0 * sigma;
  if (maxLevel <= minLevel) {
    minLevel = mean - 1.0;
    maxLevel = mean + 1.0;
  }
  double range = maxLevel - minLevel;  // Not const, needed for debug output later
  const double invRange = (range > 0.0) ? (255.0 / range) : 0.0;

  // Pre-compute lookup tables for normalization and color mapping
  // This avoids repeated calculations for each pixel
  static uint16_t colorLUT[256];
  static uint8_t normIndexLUT[256];
  for (int v = 0; v < 256; ++v) {
    double norm = (static_cast<double>(v) - minLevel) * invRange;
    if (norm < 0.0) norm = 0.0;
    if (norm > 255.0) norm = 255.0;
    uint8_t idx = static_cast<uint8_t>(norm + 0.5);
    normIndexLUT[v] = idx;
    colorLUT[v] = mapValueToColor(idx);
  }

  const int leftPad = bufferFilled ? 0 : (WATERFALL_COLUMNS - static_cast<int>(validColumns));

  // Draw each row of the waterfall
  // With COLUMN_WIDTH > 1, each measurement is drawn COLUMN_WIDTH pixels wide

  for (int y = 0; y < WATERFALL_HEIGHT; y++) {
    // Draw waterfall data across full width
    for (int screenX = 0; screenX < WATERFALL_WIDTH; screenX++) {
      // Map screen X coordinate to measurement column
      int measurementCol = screenX / COLUMN_WIDTH;

      if (!bufferFilled && measurementCol < leftPad) {
        lineBuffer[screenX] = TFT_BLACK;
        continue;
      }

      size_t bufferColumn;
      if (bufferFilled) {
        bufferColumn = (currentColumn + measurementCol) % WATERFALL_COLUMNS;
      } else {
        bufferColumn = measurementCol - leftPad;
      }

      // Ensure we don't exceed buffer bounds
      if (bufferColumn >= WATERFALL_COLUMNS) {
        lineBuffer[screenX] = TFT_BLACK;
        continue;
      }

      uint8_t rawValue = waterfallRaw[bufferColumn * WATERFALL_HEIGHT + y];
      // Use pre-computed lookup table for color mapping
      lineBuffer[screenX] = colorLUT[rawValue];
    }

    // Overlay scale tick marks on this line if needed
    const float SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2;
    const float MAX_DEPTH_M = (NUM_SAMPLES * SAMPLE_RESOLUTION) / 100.0;

    // Calculate sensible SCALE_STEP: aim for 4-6 scale lines, rounded to 0.25m increments
    float idealStep = MAX_DEPTH_M / 5.0;  // Target 5 lines
    float SCALE_STEP = roundf(idealStep / 0.25f) * 0.25f;  // Round to nearest 0.25m
    if (SCALE_STEP < 0.25f) SCALE_STEP = 0.25f;  // Minimum 0.25m steps

    // Check if this Y coordinate corresponds to a scale tick (inverted Y axis)
    for (float depth_m = 0.0; depth_m <= MAX_DEPTH_M; depth_m += SCALE_STEP) {
      int tickY = WATERFALL_HEIGHT - ((depth_m / MAX_DEPTH_M) * WATERFALL_HEIGHT);
      if (y == tickY) {
        // Draw white tick mark across entire width
        for (int x = 0; x < WATERFALL_WIDTH; x++) {
          lineBuffer[x] = TFT_WHITE;
        }
        break;
      }
    }

    // Overlay depth indicator line (red line at detected depth)
    int depthPixel = WATERFALL_HEIGHT - ((currentDepthIndex * WATERFALL_HEIGHT) / NUM_SAMPLES);
    if (y >= depthPixel - 1 && y <= depthPixel + 1) {
      // Draw red line (3 pixels thick)
      for (int x = 0; x < WATERFALL_WIDTH; x++) {
        lineBuffer[x] = TFT_RED;
      }
    }

    // Push waterfall row, but skip excluded areas (depth display and scale labels)
    // Build list of excluded regions for this row
    struct ExclusionRect {
      int x;
      int width;
    };
    ExclusionRect exclusions[MAX_SCALE_LABELS + 1];
    int numExclusions = 0;

    // Check if depth display intersects this row
    if (y >= DEPTH_DISPLAY_Y && y < (DEPTH_DISPLAY_Y + DEPTH_DISPLAY_HEIGHT)) {
      exclusions[numExclusions].x = DEPTH_DISPLAY_X;
      exclusions[numExclusions].width = DEPTH_DISPLAY_WIDTH;
      numExclusions++;
    }

    // Check if any scale labels intersect this row
    for (int i = 0; i < numScaleLabels; i++) {
      if (y >= scaleLabelRects[i].y && y < (scaleLabelRects[i].y + scaleLabelRects[i].height)) {
        exclusions[numExclusions].x = SCALE_LABEL_X;
        exclusions[numExclusions].width = scaleLabelRects[i].width;
        numExclusions++;
        break;  // Only one scale label per row
      }
    }

    // If no exclusions, draw full width
    if (numExclusions == 0) {
      tft.pushImage(0, y, WATERFALL_WIDTH, 1, lineBuffer);
    } else {
      // Sort exclusions by X position (simple bubble sort for small arrays)
      for (int i = 0; i < numExclusions - 1; i++) {
        for (int j = 0; j < numExclusions - i - 1; j++) {
          if (exclusions[j].x > exclusions[j + 1].x) {
            ExclusionRect temp = exclusions[j];
            exclusions[j] = exclusions[j + 1];
            exclusions[j + 1] = temp;
          }
        }
      }

      // Draw segments between exclusions
      int currentX = 0;
      for (int i = 0; i < numExclusions; i++) {
        int excludeStart = exclusions[i].x;
        int excludeEnd = excludeStart + exclusions[i].width;

        // Draw segment before this exclusion
        if (currentX < excludeStart) {
          int segmentWidth = excludeStart - currentX;
          tft.pushImage(currentX, y, segmentWidth, 1, lineBuffer + currentX);
        }

        currentX = excludeEnd;
      }

      // Draw final segment after last exclusion
      if (currentX < WATERFALL_WIDTH) {
        int segmentWidth = WATERFALL_WIDTH - currentX;
        tft.pushImage(currentX, y, segmentWidth, 1, lineBuffer + currentX);
      }
    }
  }

}

void drawDepthDisplay() {
  // Draw depth value in reserved area (top-right)
  // This area is excluded from waterfall rendering, so no flickering
  static float lastDisplayedDepth = -999.0f;

  // Only update if depth changed significantly
  if (fabs(currentDepth - lastDisplayedDepth) > 0.1) {
    // Clear depth display area
    tft.fillRect(DEPTH_DISPLAY_X, DEPTH_DISPLAY_Y, DEPTH_DISPLAY_WIDTH, DEPTH_DISPLAY_HEIGHT, TFT_BLACK);

    // Draw depth value
    char buf[16];
    sprintf(buf, "%.1f", currentDepth);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(3);

    // Position text in the depth display area
    tft.drawString(buf, DEPTH_DISPLAY_X + 5, DEPTH_DISPLAY_Y + 2);

    lastDisplayedDepth = currentDepth;
  }
}

void drawVerticalScale() {
  // Draw scale labels and tick lines - only once, no flicker
  // Labels are excluded from waterfall rendering via scaleLabelRects array
  static bool scaleDrawn = false;

  if (scaleDrawn) {
    return;
  }

  const float SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2;
  const float MAX_DEPTH_M = (NUM_SAMPLES * SAMPLE_RESOLUTION) / 100.0;

  // Calculate sensible SCALE_STEP: aim for 4-6 scale lines, rounded to 0.25m increments
  float idealStep = MAX_DEPTH_M / 5.0;  // Target 5 lines
  float SCALE_STEP = roundf(idealStep / 0.25f) * 0.25f;  // Round to nearest 0.25m
  if (SCALE_STEP < 0.25f) SCALE_STEP = 0.25f;  // Minimum 0.25m steps

  // Draw tick lines across entire screen
  // Inverted: max depth at top (y=0), min depth at bottom (y=240)
  for (float depth_m = 0.0; depth_m <= MAX_DEPTH_M; depth_m += SCALE_STEP) {
    int y = WATERFALL_HEIGHT - ((depth_m / MAX_DEPTH_M) * WATERFALL_HEIGHT);
    if (y >= 0 && y < WATERFALL_HEIGHT) {
      tft.drawFastHLine(0, y, SCREEN_WIDTH, TFT_WHITE);
    }
  }

  // Draw labels on left side and store their positions
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  numScaleLabels = 0;

  for (float depth_m = 0.0; depth_m <= MAX_DEPTH_M; depth_m += SCALE_STEP) {
    int y = WATERFALL_HEIGHT - ((depth_m / MAX_DEPTH_M) * WATERFALL_HEIGHT);

    // Avoid top area where depth display is
    if (y >= (DEPTH_DISPLAY_Y + DEPTH_DISPLAY_HEIGHT) && y < (WATERFALL_HEIGHT - 5)) {
      // Clear background for label
      tft.fillRect(SCALE_LABEL_X, y - 4, SCALE_LABEL_WIDTH, SCALE_LABEL_HEIGHT, TFT_BLACK);

      // Draw label
      char label[16];
      sprintf(label, "%.2f", depth_m);
      tft.drawString(label, SCALE_LABEL_X + 3, y - 4);

      // Store position for waterfall exclusion
      if (numScaleLabels < MAX_SCALE_LABELS) {
        scaleLabelRects[numScaleLabels].y = y - 4;
        scaleLabelRects[numScaleLabels].width = SCALE_LABEL_WIDTH;
        scaleLabelRects[numScaleLabels].height = SCALE_LABEL_HEIGHT;
        numScaleLabels++;
      }
    }
  }

  scaleDrawn = true;
}
