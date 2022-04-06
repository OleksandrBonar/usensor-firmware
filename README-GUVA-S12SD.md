| UV Index   | Vout (m/V) | Analog Value |
|:----------:|:----------:|:------------:|
| 0          | < 50       | < 10         |
| 1          | 227        | 46           |
| 2          | 318        | 65           |
| 3          | 408        | 83           |
| 4          | 503        | 103          |
| 5          | 606        | 124          |
| 6          | 696        | 142          |
| 7          | 795        | 162          |
| 8          | 881        | 180          |
| 9          | 976        | 200          |
| 10         | 1079       | 221          |
| 11+        | 1170+      | 240          |

## Code
If you use 5v rather than 3.3v then you will need to change the 3.3 in the SENSOR_VOLATGE value

    #define SENSOR_PIN A0
    #define SENSOR_VOLTAGE 3.3

    float analog_value = analogRead(SENSOR_PIN);
    float sensor_value = analog_value / 1024 * SENSOR_VOLTAGE;
