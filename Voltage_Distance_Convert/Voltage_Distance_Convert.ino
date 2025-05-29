#define NUM_POINTS 43
#define SENSOR_PIN A0 // Change this to the actual analog pin used

// Voltage values (in V) measured from graph
// Voltage values (in V) measured from graph

const float voltages[NUM_POINTS] = {
    1.7270110707947777, 2.2247800038486716, 1.9922157649537575, 
    2.408383260235912, 2.4573440715369714, 2.3390219824877843, 
    2.2043796230049097, 2.081977509148954, 1.9106144127853246, 
    1.7514917332552273, 1.5189275799636204, 1.6290896193992719, 
    1.3965254661076647, 1.2904436226853955, 1.184361779263126, 
    1.0864399854543934, 0.9885183628522738, 0.8905965690435411, 
    0.817155266488645, 1.514847555156852, 1.2945236474921633, 
    1.0374791741533338, 0.5723508675701193, 0.755954209560667, 
    0.715153447873144, 0.6580325869585474, 0.613151800464256, 
    0.5927515052238015, 0.5397103267027467, 0.5070697858353733, 
    0.4825892945815365, 0.4458689001140085, 0.42954845847370865, 
    0.4173078704335636, 0.3928277215929537, 0.3683475727523444, 
    0.35202713111204464, 0.33978688548512587, 0.3112265406311343, 
    0.3112265406311343, 0.30306614860437087, 0.3071463446177532, 
    0.3316268358715895
};

// Corresponding distances (in cm)
const float distances[NUM_POINTS] = {
    0.5562815055190924, 0.7802880276325083, 0.5936169702784208, 
    1.0042945497459181, 1.2283042050806139, 1.4896399255107982, 
    1.6763109828648828, 2.0123207660350038, 2.460333810261832, 
    2.908346854488661, 3.393692230253538, 3.169685708140128, 
    3.76703434496171, 4.103044128131834, 4.364382981783298, 
    4.96173475182616, 5.521751057109694, 6.23109982176671, 
    6.940451719644997, 0.3696104481650078, 0.3696104481650078, 
    0.2576071871082999, 0.145603926051592, 7.612471285985245, 
    8.172484458047498, 8.807168559628412, 9.740526979620117, 
    10.076536762790239, 10.636549934852495, 11.457905093787495, 
    12.167256991665786, 12.91394435430341, 13.548628455884323, 
    14.295315818521942, 15.004667716400231, 15.826022875335232, 
    16.57271023797285, 17.468733193205235, 18.40209161319694, 
    19.074108046315903, 20.044801931066935, 19.522121090542726, 
    17.91674310421078
};

const int NUM_POINTS = sizeof(voltages) / sizeof(voltages[0]);

// Convert voltage to distance using linear interpolation
float getDistanceFromVoltage(float voltage) {
    for (int i = 0; i < NUM_POINTS - 1; i++) {
        float v1 = voltages[i];
        float v2 = voltages[i + 1];

        // Check if voltage is between v1 and v2 (or vice versa, in case of decreasing order)
        if ((voltage <= v1 && voltage >= v2) || (voltage >= v1 && voltage <= v2)) {
            float d1 = distances[i];
            float d2 = distances[i + 1];

            // Linear interpolation formula
            return d1 + (voltage - v1) * (d2 - d1) / (v2 - v1);
        }
    }

    // Voltage out of range
    return -1.0;
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12); // Arduino Giga uses 12-bit ADC: values from 0–4095
}

void loop() {
    int rawValue = analogRead(SENSOR_PIN);
    float voltage = rawValue * (3.3 / 4095.0); // Convert to volts assuming 3.3V ref

    float distance = getDistanceFromVoltage(voltage);

    Serial.print("Raw ADC: ");
    Serial.print(rawValue);
    Serial.print(" | Voltage: ");
    Serial.print(voltage, 3);
    Serial.print(" V | Distance: ");

    if (distance >= 0)
        Serial.print(distance, 2);
    else
        Serial.print("Out of range");

    Serial.println(" cm");

    delay(500);
}