#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width
#define SCREEN_HEIGHT 64 // OLED display height
#define OLED_RESET    -1 // Reset pin
#define BUZZER_PIN 8     // Buzzer pin

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MAX30105 particleSensor;

const byte RATE_SIZE = 2; // Reduced to save SRAM
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 1000;

float beatsPerMinute;
byte beatAvg; // Byte to save SRAM

// Heart icon bitmap (large heart, 32x32 pixels)
static const unsigned char PROGMEM heart_large_bmp[] = {
  0x07, 0xFC, 0x3F, 0xE0, 0x1F, 0xFF, 0xFF, 0xF8, 0x7E, 0x1F, 0xF8, 0x7E, 0x78, 0x07, 0xE0, 0x1E,
  0xF0, 0x07, 0xE0, 0x0F, 0xE0, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x07, 0xF0, 0x00, 0x3E, 0x0F,
  0xE0, 0x00, 0x3E, 0x07, 0xE0, 0x00, 0x7E, 0x07, 0xE0, 0x00, 0x7F, 0x07, 0xE0, 0x00, 0x6F, 0x00,
  0xE0, 0x00, 0x6F, 0x00, 0xE0, 0x00, 0x6F, 0x00, 0xF0, 0x78, 0x67, 0x00, 0xF0, 0x78, 0x67, 0x00,
  0xFF, 0xFF, 0xCF, 0xFE, 0x0F, 0xFF, 0xC0, 0x1E, 0x07, 0xFF, 0xC0, 0x3E, 0x03, 0xFF, 0xC0, 0x3C,
  0x01, 0xE3, 0xE0, 0x78, 0x00, 0xE1, 0xF0, 0xF0, 0x1F, 0xC0, 0xE1, 0xF0, 0x0F, 0xE0, 0xE3, 0xE0,
  0x07, 0xF0, 0x07, 0xC0, 0x03, 0xF8, 0x0F, 0x80, 0x01, 0xFC, 0x1F, 0x00, 0x00, 0xFE, 0x3E, 0x00,
  0x00, 0x3F, 0xF8, 0x00, 0x00, 0x1F, 0xF0, 0x00, 0x00, 0x0F, 0xE0, 0x00, 0x00, 0x07, 0xC0, 0x00
};

// Heart icon bitmap (small heart, 32x32 pixels)
static const unsigned char PROGMEM heart_small_bmp[] = {
  0x01, 0xF0, 0x0F, 0x80, 0x06, 0x1C, 0x38, 0x60, 0x18, 0x06, 0x60, 0x18, 0x10, 0x01, 0x80, 0x08,
  0x20, 0x01, 0x80, 0x04, 0x40, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0xC0, 0x00, 0x08, 0x03,
  0x80, 0x00, 0x08, 0x01, 0x80, 0x00, 0x18, 0x01, 0x80, 0x00, 0x1C, 0x01, 0x80, 0x00, 0x14, 0x00,
  0x80, 0x00, 0x14, 0x00, 0x80, 0x00, 0x14, 0x00, 0x40, 0x10, 0x12, 0x00, 0x40, 0x10, 0x12, 0x00,
  0x7E, 0x1F, 0x23, 0xFE, 0x03, 0x31, 0xA0, 0x04, 0x01, 0xA0, 0xA0, 0x0C, 0x00, 0xA0, 0xA0, 0x08,
  0x00, 0x60, 0xE0, 0x10, 0x00, 0x20, 0x60, 0x20, 0x06, 0x00, 0x40, 0x60, 0x03, 0x00, 0x40, 0xC0,
  0x01, 0x80, 0x01, 0x80, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x30, 0x0C, 0x00,
  0x00, 0x08, 0x10, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x01, 0x80, 0x00
};

// ML probability arrays (in PROGMEM)
const double PROGMEM PROB_0_5_0_5[2] = {0.5, 0.5};
const double PROGMEM PROB_0_0_1_0[2] = {0.0, 1.0};
const double PROGMEM PROB_1_0_0_0[2] = {1.0, 0.0};
const double PROGMEM PROB_0_333_0_667[2] = {0.3333333333333333, 0.6666666666666666};
const double PROGMEM PROB_0_667_0_333[2] = {0.6666666666666666, 0.3333333333333333};
const double PROGMEM PROB_0_25_0_75[2] = {0.25, 0.75};
const double PROGMEM PROB_0_538_0_462[2] = {0.5384615384615384, 0.46153846153846156};
const double PROGMEM PROB_0_462_0_538[2] = {0.46153846153846156, 0.5384615384615384};
const double PROGMEM PROB_0_392_0_608[2] = {0.3918918918918919, 0.6081081081081081};
const double PROGMEM PROB_0_36_0_64[2] = {0.36, 0.64};
const double PROGMEM PROB_0_385_0_615[2] = {0.38461538461538464, 0.6153846153846154};
const double PROGMEM PROB_0_391_0_609[2] = {0.391304347826087, 0.6086956521739131};
const double PROGMEM PROB_0_310_0_690[2] = {0.30952380952380953, 0.6904761904761905};
const double PROGMEM PROB_0_3_0_7[2] = {0.3, 0.7};
const double PROGMEM PROB_0_519_0_481[2] = {0.5185185185185185, 0.48148148148148145};
const double PROGMEM PROB_0_167_0_833[2] = {0.16666666666666666, 0.8333333333333334};
const double PROGMEM PROB_0_436_0_564[2] = {0.4358974358974359, 0.5641025641025641};
const double PROGMEM PROB_0_455_0_545[2] = {0.45454545454545453, 0.5454545454545454};
const double PROGMEM PROB_0_231_0_769[2] = {0.23076923076923078, 0.7692307692307693};
const double PROGMEM PROB_0_32_0_68[2] = {0.32, 0.68};
const double PROGMEM PROB_0_314_0_686[2] = {0.3142857142857143, 0.6857142857142857};
const double PROGMEM PROB_0_476_0_524[2] = {0.47619047619047616, 0.5238095238095238};
const double PROGMEM PROB_0_409_0_591[2] = {0.4090909090909091, 0.5909090909090909};
const double PROGMEM PROB_0_471_0_529[2] = {0.47058823529411764, 0.5294117647058824};
const double PROGMEM PROB_0_478_0_522[2] = {0.4782608695652174, 0.5217391304347826};
const double PROGMEM PROB_0_591_0_409[2] = {0.5909090909090909, 0.4090909090909091};
const double PROGMEM PROB_0_405_0_595[2] = {0.40540540540540543, 0.5945945945945946};
const double PROGMEM PROB_0_387_0_613[2] = {0.3870967741935484, 0.6129032258064516};
const double PROGMEM PROB_0_4_0_6[2] = {0.4, 0.6};
const double PROGMEM PROB_0_375_0_625[2] = {0.375, 0.625};
const double PROGMEM PROB_0_556_0_444[2] = {0.5555555555555556, 0.4444444444444444};
const double PROGMEM PROB_0_308_0_692[2] = {0.3076923076923077, 0.6923076923076923};
const double PROGMEM PROB_0_615_0_385[2] = {0.6153846153846154, 0.38461538461538464};
const double PROGMEM PROB_0_371_0_629[2] = {0.37142857142857144, 0.6285714285714286};
const double PROGMEM PROB_0_3125_0_6875[2] = {0.3125, 0.6875};
const double PROGMEM PROB_0_222_0_778[2] = {0.2222222222222222, 0.7777777777777778};
const double PROGMEM PROB_0_444_0_556[2] = {0.4444444444444444, 0.5555555555555556};
const double PROGMEM PROB_0_273_0_727[2] = {0.2727272727272727, 0.7272727272727273};
const double PROGMEM PROB_0_542_0_458[2] = {0.5416666666666666, 0.4583333333333333};
const double PROGMEM PROB_0_235_0_765[2] = {0.23529411764705882, 0.7647058823529411};
const double PROGMEM PROB_0_2_0_8[2] = {0.2, 0.8};

bool heartState = false;
bool warningActive = false;
byte dangerousCount = 0;
byte normalCount = 0;

// ML scoring function
void score(double *input, double *output) {
  if (input[0] <= 0.33255474269390106) {
    if (input[0] <= 0.2938030809164047) {
      if (input[0] <= 0.21629977226257324) {
        if (input[0] <= -0.0743376687169075) {
          if (input[0] <= -0.1324651539325714) {
            if (input[0] <= -0.42310260236263275) {
              if (input[0] <= -0.5006059110164642) {
                if (input[0] <= -0.5199817419052124) {
                  if (input[0] <= -0.5974850654602051) {
                    if (input[0] <= -0.6362367272377014) {
                      if (input[0] <= -0.6943641901016235) {
                        if (input[0] <= -0.9753136932849884) {
                          memcpy_P(output, PROB_0_5_0_5, 2 * sizeof(double));
                        } else {
                          if (input[0] <= -0.7815554141998291) {
                            memcpy_P(output, PROB_0_0_1_0, 2 * sizeof(double));
                          } else {
                            memcpy_P(output, PROB_0_5_0_5, 2 * sizeof(double));
                          }
                        }
                      } else {
                        memcpy_P(output, PROB_0_0_1_0, 2 * sizeof(double));
                      }
                    } else {
                      memcpy_P(output, PROB_1_0_0_0, 2 * sizeof(double));
                    }
                  } else {
                    if (input[0] <= -0.5393575727939606) {
                      memcpy_P(output, PROB_0_0_1_0, 2 * sizeof(double));
                    } else {
                      memcpy_P(output, PROB_0_333_0_667, 2 * sizeof(double));
                    }
                  }
                } else {
                  memcpy_P(output, PROB_0_667_0_333, 2 * sizeof(double));
                }
              } else {
                if (input[0] <= -0.4618542641401291) {
                  if (input[0] <= -0.48123008012771606) {
                    memcpy_P(output, PROB_0_0_1_0, 2 * sizeof(double));
                  } else {
                    memcpy_P(output, PROB_0_5_0_5, 2 * sizeof(double));
                  }
                } else {
                  if (input[0] <= -0.4424784332513809) {
                    memcpy_P(output, PROB_0_0_1_0, 2 * sizeof(double));
                  } else {
                    memcpy_P(output, PROB_0_25_0_75, 2 * sizeof(double));
                  }
                }
              }
            } else {
              if (input[0] <= -0.36497510969638824) {
                if (input[0] <= -0.4037267714738846) {
                  memcpy_P(output, PROB_0_538_0_462, 2 * sizeof(double));
                } else {
                  if (input[0] <= -0.3843509405851364) {
                    memcpy_P(output, PROB_0_462_0_538, 2 * sizeof(double));
                  } else {
                    memcpy_P(output, PROB_0_5_0_5, 2 * sizeof(double));
                  }
                }
              } else {
                if (input[0] <= -0.22934430092573166) {
                  if (input[0] <= -0.28747178614139557) {
                    if (input[0] <= -0.3455992788076401) {
                      memcpy_P(output, PROB_0_392_0_608, 2 * sizeof(double));
                    } else {
                      if (input[0] <= -0.3262234479188919) {
                        memcpy_P(output, PROB_0_36_0_64, 2 * sizeof(double));
                      } else {
                        if (input[0] <= -0.30684761703014374) {
                          memcpy_P(output, PROB_0_385_0_615, 2 * sizeof(double));
                        } else {
                          memcpy_P(output, PROB_0_391_0_609, 2 * sizeof(double));
                        }
                      }
                    }
                  } else {
                    if (input[0] <= -0.24872012436389923) {
                      if (input[0] <= -0.2680959552526474) {
                        memcpy_P(output, PROB_0_310_0_690, 2 * sizeof(double));
                      } else {
                        memcpy_P(output, PROB_0_36_0_64, 2 * sizeof(double));
                      }
                    } else {
                      memcpy_P(output, PROB_0_3_0_7, 2 * sizeof(double));
                    }
                  }
                } else {
                  if (input[0] <= -0.2099684774875641) {
                    if (input[0] <= -0.19059264659881592) {
                      memcpy_P(output, PROB_0_167_0_833, 2 * sizeof(double));
                    } else {
                      memcpy_P(output, PROB_0_519_0_481, 2 * sizeof(double));
                    }
                  } else {
                    if (input[0] <= -0.17121681571006775) {
                      memcpy_P(output, PROB_0_462_0_538, 2 * sizeof(double));
                    } else {
                      if (input[0] <= -0.15184098482131958) {
                        memcpy_P(output, PROB_0_436_0_564, 2 * sizeof(double));
                      } else {
                        memcpy_P(output, PROB_0_455_0_545, 2 * sizeof(double));
                      }
                    }
                  }
                }
              }
            }
          } else {
            if (input[0] <= -0.11308932304382324) {
              memcpy_P(output, PROB_0_231_0_769, 2 * sizeof(double));
            } else {
              if (input[0] <= -0.09371349588036537) {
                memcpy_P(output, PROB_0_32_0_68, 2 * sizeof(double));
              } else {
                memcpy_P(output, PROB_0_314_0_686, 2 * sizeof(double));
              }
            }
          }
        } else {
          if (input[0] <= 0.022541478741914034) {
            if (input[0] <= 0.003165649250149727) {
              if (input[0] <= -0.03558600880205631) {
                if (input[0] <= -0.05496183782815933) {
                  memcpy_P(output, PROB_0_476_0_524, 2 * sizeof(double));
                } else {
                  memcpy_P(output, PROB_0_409_0_591, 2 * sizeof(double));
                }
              } else {
                if (input[0] <= -0.01621018024161458) {
                  memcpy_P(output, PROB_0_471_0_529, 2 * sizeof(double));
                } else {
                  memcpy_P(output, PROB_0_478_0_522, 2 * sizeof(double));
                }
              }
            } else {
              memcpy_P(output, PROB_0_591_0_409, 2 * sizeof(double));
            }
          } else {
            if (input[0] <= 0.1775481104850769) {
              if (input[0] <= 0.10004479438066483) {
                if (input[0] <= 0.06129313446581364) {
                  if (input[0] <= 0.041917307302355766) {
                    memcpy_P(output, PROB_0_405_0_595, 2 * sizeof(double));
                  } else {
                    memcpy_P(output, PROB_0_387_0_613, 2 * sizeof(double));
                  }
                } else {
                  if (input[0] <= 0.08066896349191666) {
                    memcpy_P(output, PROB_0_436_0_564, 2 * sizeof(double));
                  } else {
                    memcpy_P(output, PROB_0_4_0_6, 2 * sizeof(double));
                  }
                }
              } else {
                if (input[0] <= 0.13879644870758057) {
                  if (input[0] <= 0.1194206215441227) {
                    memcpy_P(output, PROB_0_375_0_625, 2 * sizeof(double));
                  } else {
                    memcpy_P(output, PROB_0_25_0_75, 2 * sizeof(double));
                  }
                } else {
                  if (input[0] <= 0.15817227959632874) {
                    memcpy_P(output, PROB_0_556_0_444, 2 * sizeof(double));
                  } else {
                    memcpy_P(output, PROB_0_308_0_692, 2 * sizeof(double));
                  }
                }
              }
            } else {
              if (input[0] <= 0.19692394137382507) {
                memcpy_P(output, PROB_0_615_0_385, 2 * sizeof(double));
              } else {
                memcpy_P(output, PROB_0_371_0_629, 2 * sizeof(double));
              }
            }
          }
        }
      } else {
        if (input[0] <= 0.255051426589489) {
          if (input[0] <= 0.2356756031513214) {
            memcpy_P(output, PROB_0_3125_0_6875, 2 * sizeof(double));
          } else {
            memcpy_P(output, PROB_0_222_0_778, 2 * sizeof(double));
          }
        } else {
          if (input[0] <= 0.27442725002765656) {
            memcpy_P(output, PROB_0_444_0_556, 2 * sizeof(double));
          } else {
            memcpy_P(output, PROB_0_273_0_727, 2 * sizeof(double));
          }
        }
      }
    } else {
      if (input[0] <= 0.3131789118051529) {
        memcpy_P(output, PROB_0_542_0_458, 2 * sizeof(double));
      } else {
        memcpy_P(output, PROB_0_5_0_5, 2 * sizeof(double));
      }
    }
  } else {
    if (input[0] <= 0.3713064044713974) {
      if (input[0] <= 0.35193057358264923) {
        memcpy_P(output, PROB_0_235_0_765, 2 * sizeof(double));
      } else {
        memcpy_P(output, PROB_0_0_1_0, 2 * sizeof(double));
      }
    } else {
      if (input[0] <= 0.4294338971376419) {
        if (input[0] <= 0.39068223536014557) {
          memcpy_P(output, PROB_0_333_0_667, 2 * sizeof(double));
        } else {
          if (input[0] <= 0.41005806624889374) {
            memcpy_P(output, PROB_1_0_0_0, 2 * sizeof(double));
          } else {
            memcpy_P(output, PROB_0_4_0_6, 2 * sizeof(double));
          }
        }
      } else {
        if (input[0] <= 0.6038163602352142) {
          if (input[0] <= 0.46818555891513824) {
            if (input[0] <= 0.4488097280263901) {
              memcpy_P(output, PROB_0_2_0_8, 2 * sizeof(double));
            } else {
              memcpy_P(output, PROB_0_0_1_0, 2 * sizeof(double));
            }
          } else {
            if (input[0] <= 0.5360009372234344) {
              if (input[0] <= 0.4972492903470993) {
                memcpy_P(output, PROB_0_273_0_727, 2 * sizeof(double));
              } else {
                memcpy_P(output, PROB_0_333_0_667, 2 * sizeof(double));
              }
            } else {
              if (input[0] <= 0.5650646984577179) {
                memcpy_P(output, PROB_0_0_1_0, 2 * sizeof(double));
              } else {
                memcpy_P(output, PROB_0_25_0_75, 2 * sizeof(double));
              }
            }
          }
        } else {
          if (input[0] <= 0.8750779628753662) {
            if (input[0] <= 0.6619438529014587) {
              if (input[0] <= 0.6425680220127106) {
                memcpy_P(output, PROB_0_667_0_333, 2 * sizeof(double));
              } else {
                memcpy_P(output, PROB_0_667_0_333, 2 * sizeof(double));
              }
            } else {
              if (input[0] <= 0.6813196539878845) {
                memcpy_P(output, PROB_0_0_1_0, 2 * sizeof(double));
              } else {
                if (input[0] <= 0.739447146654129) {
                  memcpy_P(output, PROB_1_0_0_0, 2 * sizeof(double));
                } else {
                  if (input[0] <= 0.8266383707523346) {
                    if (input[0] <= 0.7975746393203735) {
                      if (input[0] <= 0.7685109078884125) {
                        memcpy_P(output, PROB_0_25_0_75, 2 * sizeof(double));
                      } else {
                        memcpy_P(output, PROB_0_667_0_333, 2 * sizeof(double));
                      }
                    } else {
                      memcpy_P(output, PROB_0_0_1_0, 2 * sizeof(double));
                    }
                  } else {
                    memcpy_P(output, PROB_1_0_0_0, 2 * sizeof(double));
                  }
                }
              }
            }
          } else {
            if (input[0] <= 10.553304493427277) {
              if (input[0] <= 1.0591483116149902) {
                if (input[0] <= 0.9719571173191071) {
                  memcpy_P(output, PROB_0_25_0_75, 2 * sizeof(double));
                } else {
                  memcpy_P(output, PROB_0_333_0_667, 2 * sizeof(double));
                }
              } else {
                memcpy_P(output, PROB_0_0_1_0, 2 * sizeof(double));
              }
            } else {
              memcpy_P(output, PROB_0_5_0_5, 2 * sizeof(double));
            }
          }
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Initializing..."));

  // Initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialize I2C
  Wire.begin();
  Wire.setClock(100000);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1);
  }

  display.ssd1306_command(SSD1306_DISPLAYON);
  display.setRotation(0);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 10);
  display.println(F("Horus"));
  display.setCursor(30, 30);
  display.println(F(" Heart"));
  display.display();
  delay(2000);

  // Initialize MAX30105
  if (!particleSensor.begin(Wire, 100000)) {
    Serial.println(F("MAX30105 not found."));
    while (1);
  }
  Serial.println(F("Place finger on sensor."));

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
}

void loop() {
  long irValue = particleSensor.getIR();

  if (irValue < 50000) {
    beatsPerMinute = 0;
    beatAvg = 0;
    memset(rates, 0, sizeof(rates));
    rateSpot = 0;
    dangerousCount = 0;
    normalCount = 0;
    warningActive = false;
  } else if (checkForBeat(irValue)) {
    Serial.println(F("Beat detected!"));
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  // Update OLED every 1 second
  if (millis() - lastDisplayUpdate >= displayInterval) {
    display.clearDisplay();

    double heartAttackProb = 0.0;
    if (beatAvg > 0) {
      double normalizedBPM = (beatAvg - 78.33661865) / 51.61069455;
      double input[1] = {normalizedBPM};
      double output[2];
      score(input, output);
      heartAttackProb = output[1];

      // Update warning counters
      if (heartAttackProb > 0.8 || beatAvg <= 40 || beatAvg >= 120) {
        dangerousCount++;
        normalCount = 0;
        if (dangerousCount >= 5) {
          warningActive = true;
        }
      } else {
        normalCount++;
        dangerousCount = 0;
        if (normalCount >= 5) {
          warningActive = false;
        }
      }
    } else {
      dangerousCount = 0;
      normalCount = 0;
      warningActive = false;
    }

    if (irValue < 50000) {
      display.setTextSize(1);
      display.setCursor(20, 20);
      display.println(F("Place finger"));
      display.setCursor(35, 35);
      display.println(F("on sensor"));
      digitalWrite(BUZZER_PIN, LOW);
    } else if (beatAvg == 0) {
      display.setTextSize(1);
      display.setCursor(30, 25);
      display.println(F("Measuring..."));
      digitalWrite(BUZZER_PIN, LOW);
    } else {
      if (warningActive) {
        display.setTextSize(1);
        display.setCursor(10, 20);
        display.println(F("WARNING: High"));
        display.setCursor(10, 35);
        display.println(F("probability!"));
        digitalWrite(BUZZER_PIN, HIGH);
      } else {
        display.setTextSize(2);
        display.setCursor(10, 10);
        display.print(F("BPM: "));
        display.println(beatAvg);
        heartState = !heartState;
        display.drawBitmap(48, 28, heartState ? heart_large_bmp : heart_small_bmp, 32, 32, SSD1306_WHITE);
        digitalWrite(BUZZER_PIN, LOW);
      }
    }

    display.display();
    lastDisplayUpdate = millis();

    // ML debugging
    if (beatAvg > 0) {
      Serial.print(F("Heart Attack Prob="));
      Serial.print(heartAttackProb, 3);
      if (heartAttackProb > 0.8 || beatAvg <= 40 || beatAvg >= 120) {
        Serial.print(F(" [DANGEROUS, Count: "));
        Serial.print(dangerousCount);
        Serial.print(F("]"));
      } else {
        Serial.print(F(" [Normal, Count: "));
        Serial.print(normalCount);
        Serial.print(F("]"));
      }
      Serial.println();
    }
  }

  // Serial output for debugging
  Serial.print(F("IR="));
  Serial.print(irValue);
  Serial.print(F(", BPM="));
  Serial.print(beatsPerMinute);
  Serial.print(F(", Avg BPM="));
  Serial.print(beatAvg);
  if (irValue < 50000)
    Serial.print(F(" No finger?"));
  Serial.println();
}